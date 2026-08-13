#!/bin/bash
# Interactive HILS demo: the Hokuyo YVT-35LX normal path, kept running
# and shown in RViz.
#
#   scene source -> yvt35lx emulator -> VSSP TCP 127.0.0.1:10940
#       -> urg3d_node2 -> /hokuyo_cloud2 + /imu -> RViz
#
# run_yvt35lx_e2e.sh exits as soon as the oracle has judged the
# scenario, which is what CI wants but leaves nothing to look at. This
# script brings the same chain up and then stays out of the way until
# you close RViz or press Ctrl-C, so the stream can be watched - and
# faults injected by hand (see the banner it prints).
#
# No hardware and no simulator: the point source is a built-in scene
# (a street seen from a 2 m mount) unless DEMO_BAG points at a
# PointCloud2 rosbag.
#
# Requires: a built workspace (hils_bridge_base / hils_bridge_interfaces
# / hils_bridge_lidar_hokuyo_yvt35lx / hils_bringup / urg3d_node2) and a
# reachable X display for RViz. In docker, docker/launch_docker.sh
# already forwards the X socket.
#
# Stop it with Ctrl-C, or by closing RViz. If you background the script
# instead of running it in a terminal, use SIGTERM: a shell that starts
# a background job makes it ignore SIGINT, and an ignored signal cannot
# be trapped, so Ctrl-C's signal would never reach the cleanup.
#
# Environment:
#   DEMO_BAG=<dir>        replay a rosbag instead of the built-in scene
#   DEMO_BAG_TOPIC=<t>    topic to remap from the bag (default /livox/lidar)
#   DEMO_RVIZ=0           headless: bring the chain up without RViz
#   DEMO_SCAN_HZ=<hz>     frames per second (default 20, the datasheet rate)
#   DEMO_DOMAIN_ID=<id>   ROS domain to run in (default 72)
#   DEMO_NOISE_SIGMA=<m>  range noise std-dev (default 0.02; 0 disables)
#   DEMO_DROPOUT=<p>      spots reporting no echo (default 0.01)
#   DEMO_NOISE_SEED=<n>   noise seed (default 1)
#   DEMO_MOUNT=upright    mount the sensor the other way up (default
#                         inverted: -35..+5 deg, looking down at the
#                         ground, as in Hokuyo's own demo footage)
#   DEMO_INTERLACE_H=<n>  horizontal interlace the driver asks for (1-20)
#   DEMO_INTERLACE_V=<n>  vertical interlace the driver asks for (1-10)
set -u

export ROS_AUTOMATIC_DISCOVERY_RANGE="${DEMO_DISCOVERY_RANGE:-LOCALHOST}"
# Own domain by default, for the same reason as the E2E script: nodes
# left over from another run share our node and service names. 72 keeps
# the demo clear of the E2E's 71, so both can run at once.
export ROS_DOMAIN_ID="${DEMO_DOMAIN_ID:-${ROS_DOMAIN_ID:-72}}"

WORK=$(mktemp -d)
SCAN_HZ="${DEMO_SCAN_HZ:-20.0}"
WITH_RVIZ="${DEMO_RVIZ:-1}"
# Force a decimal point: ROS types "0" as an integer and then refuses to
# assign it to a double parameter, so DEMO_DROPOUT=0 - the documented way
# to switch dropout off - would kill the emulator at startup.
NOISE_SIGMA=$(printf '%.6f' "${DEMO_NOISE_SIGMA:-0.02}" 2>/dev/null) || {
    echo "[demo] FAIL: DEMO_NOISE_SIGMA must be a number"; exit 3; }
DROPOUT=$(printf '%.6f' "${DEMO_DROPOUT:-0.01}" 2>/dev/null) || {
    echo "[demo] FAIL: DEMO_DROPOUT must be a number"; exit 3; }
# Mounting. The sensor scans -5..+35 deg about its own axis, so hanging
# it upside down turns that into -35..+5 and it looks at the ground -
# which is how the vendor's demo videos are shot, and the only way the
# ground shows up from 1.4x to 11x the mounting height.
if [ "${DEMO_MOUNT:-inverted}" = upright ]; then
    V_MIN=-5.0; V_MAX=35.0
else
    V_MIN=-35.0; V_MAX=5.0
fi
# Interlace is requested by the driver, not the sensor. Each field is a
# separate scan, and urg3d_node2 accumulates interlace_h x interlace_v
# of them into one published cloud - so this multiplies both the point
# count and the time the cloud takes to build.
INTERLACE_H="${DEMO_INTERLACE_H:-2}"
INTERLACE_V="${DEMO_INTERLACE_V:-2}"
PIDS=()

if ! ros2 pkg prefix urg3d_node2 >/dev/null 2>&1; then
    echo '[demo] FAIL: urg3d_node2 is not built in this workspace.'
    echo '       The docker image clones it into ~/colcon_ws/src; otherwise'
    echo '       git clone --recursive https://github.com/Hokuyo-aut/urg3d_node2'
    exit 3
fi
if [ "$WITH_RVIZ" != "0" ] && [ -z "${DISPLAY:-}" ]; then
    echo '[demo] FAIL: DISPLAY is not set, so RViz cannot open a window.'
    echo '       Use docker/launch_docker.sh (it forwards the X socket),'
    echo '       or run headless with DEMO_RVIZ=0.'
    exit 3
fi

cleanup() {
    echo '[demo] shutting down ...'
    # `kill -- -$pid` alone is not enough: with setsid in front, $! is
    # the wrapper's pid and the real process group ends up with another
    # id, which is how this repo's E2E scripts have been leaving orphan
    # drivers behind. Kill by command line as well. Everything we start
    # either carries the unique $WORK path or has a name of its own.
    for pid in "${PIDS[@]}"; do
        kill -- -"$pid" 2>/dev/null || kill "$pid" 2>/dev/null
    done
    # Only two patterns, to keep the blast radius small: $WORK is unique
    # to this run, and urg3d_node2 has to be named because it can
    # outlive the group kill (below). Matching on more generic names
    # would also hit another shell that merely mentions them.
    pkill -f "$WORK" 2>/dev/null
    # urg3d_node2 does not always honour SIGTERM: its scan thread can be
    # blocked on the socket, so the join in its destructor never returns.
    # Target the pid we started, never the name - killing by name here
    # once took down the driver of an E2E that was starting up next to
    # this demo.
    if [ -n "${DRIVER_NODE:-}" ]; then
        kill "$DRIVER_NODE" 2>/dev/null
        for _ in 1 2 3 4 5; do
            kill -0 "$DRIVER_NODE" 2>/dev/null || break
            sleep 1
        done
        kill -9 "$DRIVER_NODE" 2>/dev/null
    fi
    pkill -9 -f "$WORK" 2>/dev/null
    wait 2>/dev/null
    echo "[demo] logs kept in $WORK"
}
trap cleanup EXIT INT TERM

echo "[demo] workdir: $WORK (domain $ROS_DOMAIN_ID," \
     "discovery $ROS_AUTOMATIC_DISCOVERY_RANGE)"

# ── 1. Point source ──
if [ -n "${DEMO_BAG:-}" ]; then
    echo "[demo] point source: bag $DEMO_BAG"
    setsid ros2 bag play "$DEMO_BAG" --loop \
        --remap "${DEMO_BAG_TOPIC:-/livox/lidar}:=/sim_points" \
        > "$WORK/source.log" 2>&1 &
    PIDS+=($!)
else
    echo '[demo] point source: built-in scene (street, sensor 2 m up looking down)'
    # Written out rather than piped in on stdin: the command line then
    # carries the unique $WORK path, which is what cleanup matches on.
    cat > "$WORK/scene_source.py" <<'EOF'
# A street-like scene for a sensor hanging 2 m up and looking down
# (-35..+5 deg): ground from 2.9 m out, a kerb, a facade either side and
# a person-sized post. Each surface gets its own intensity so RViz
# colours them apart. On the ground this draws the sensor's radial
# traces - one per vertical oscillation - which is what the real device
# shows.
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

MOUNT_HEIGHT = 2.0


def frange(start, stop, step):
    value = start
    while value <= stop + 1e-9:
        yield value
        value += step


pts = []
for x in frange(-24.0, 24.0, 0.05):        # road surface
    for y in frange(-10.0, 10.0, 0.05):
        if 2.0 < math.hypot(x, y) <= 26.0:
            pts.append((x, y, -MOUNT_HEIGHT, 60.0))
for x in frange(-24.0, 24.0, 0.03):        # kerbs
    for side in (-4.0, 4.0):
        for z in frange(-MOUNT_HEIGHT, -MOUNT_HEIGHT + 0.15, 0.03):
            pts.append((x, side, z, 150.0))
for x in frange(-24.0, 24.0, 0.05):        # facades behind the kerbs
    for side in (-8.0, 8.0):
        for z in frange(-MOUNT_HEIGHT, 1.0, 0.05):
            pts.append((x, side, z, 200.0))
for z in frange(-MOUNT_HEIGHT, -0.3, 0.02):   # a person 6 m ahead
    for a in frange(0.0, 6.28, 0.15):
        pts.append((6.0 + 0.2 * math.cos(a),
                    1.0 + 0.2 * math.sin(a), z, 400.0))

rclpy.init()
node = Node('yvt_scene_source')
pub = node.create_publisher(PointCloud2, '/sim_points', 5)

msg = PointCloud2()
msg.header.frame_id = 'hokuyo3d'
msg.height = 1
msg.width = len(pts)
msg.fields = [
    PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
    for n, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
msg.is_bigendian = False
msg.point_step = 16
msg.row_step = 16 * len(pts)
msg.is_dense = True
msg.data = b''.join(struct.pack('<ffff', *p) for p in pts)
node.get_logger().info(f'scene: {len(pts)} points')


def tick():
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)


node.create_timer(0.1, tick)
rclpy.spin(node)
EOF
    setsid python3 "$WORK/scene_source.py" > "$WORK/source.log" 2>&1 &
    PIDS+=($!)
fi

# The cloud is drawn in its own frame, so RViz needs no transform - but
# with no TF tree at all it raises a global status warning.
setsid ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id world --child-frame-id hokuyo3d > "$WORK/tf.log" 2>&1 &
PIDS+=($!)

# ── 2. Emulator and the real driver ──
# Sensor noise is off in the emulator's defaults so it cannot double up
# with a simulation that models its own. The demo's point source is
# noise-free, so turn it on here to show what the driver sees off a real
# device: a wall thickens to the noise band instead of being a perfect
# plane, and a few spots report nothing.
setsid ros2 run hils_bridge_lidar_hokuyo_yvt35lx yvt35lx_emulator_node \
    --ros-args \
    -p device_ip:=127.0.0.1 -p host_ip:=127.0.0.1 \
    -p pointcloud_topic:=/sim_points -p scan_rate_hz:="$SCAN_HZ" \
    -p vertical_fov_min_deg:="$V_MIN" -p vertical_fov_max_deg:="$V_MAX" \
    -p range_noise_sigma_m:="$NOISE_SIGMA" \
    -p dropout_probability:="$DROPOUT" \
    -p noise_seed:="${DEMO_NOISE_SEED:-1}" \
    > "$WORK/emulator.log" 2>&1 &
PIDS+=($!)
sleep 3

setsid ros2 run urg3d_node2 urg3d_node2_node --ros-args \
    -p ip_address:=127.0.0.1 -p ip_port:=10940 \
    -p frame_id:=hokuyo3d \
    -p interlace_h:="$INTERLACE_H" -p interlace_v:="$INTERLACE_V" \
    -p publish_intensity:=true -p publish_auxiliary:=true \
    > "$WORK/driver.log" 2>&1 &
DRIVER_RUNNER=$!
PIDS+=($DRIVER_RUNNER)
sleep 3
# The node is a child of `ros2 run`. Remember its pid: cleanup has to
# kill this one process rather than everything named urg3d_node2_node,
# which would also take down an E2E or a real driver running alongside.
DRIVER_NODE=$(pgrep -P "$DRIVER_RUNNER" 2>/dev/null | head -1)

# `ros2 lifecycle set` can exit 0 while printing "Transitioning failed",
# so verify the reached state instead of trusting the exit code.
for step in configure:inactive activate:active; do
    transition="${step%%:*}"
    want="${step##*:}"
    OK=0
    for _ in 1 2 3 4 5; do
        ros2 lifecycle set /urg3d_node2 "$transition" \
            >> "$WORK/driver.log" 2>&1
        state=$(ros2 lifecycle get /urg3d_node2 2>/dev/null)
        case "$state" in "$want"*) OK=1; break ;; esac
        sleep 2
    done
    if [ "$OK" -ne 1 ]; then
        echo "[demo] FAIL: lifecycle $transition did not reach $want (state: ${state:-unknown})"
        tail -n 20 "$WORK/driver.log" "$WORK/emulator.log"
        exit 2
    fi
done

# ── 3. Wait for the driver's own topic ──
python3 - <<'EOF' || {
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('hils_demo_probe')
got = []
node.create_subscription(PointCloud2, '/hokuyo_cloud2',
                         lambda m: got.append(m), 10)
end = time.monotonic() + 60.0
while time.monotonic() < end and not got:
    rclpy.spin_once(node, timeout_sec=0.5)
if got:
    print(f'[demo] /hokuyo_cloud2 up: {got[-1].width} points/cloud, '
          f'frame {got[-1].header.frame_id}')
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
EOF
    echo '[demo] FAIL: /hokuyo_cloud2 never appeared within 60s'
    tail -n 20 "$WORK/driver.log" "$WORK/emulator.log"
    exit 2
}

cat <<BANNER

  ── YVT-35LX HILS demo (domain $ROS_DOMAIN_ID) ─────────────────────
  Point cloud : /hokuyo_cloud2   (frame hokuyo3d, ~${SCAN_HZ} Hz)
  Auxiliary   : /imu

  Each of the 35 measured lines is one 1200 Hz oscillation of 74 spots
  drifting ~6 deg across in azimuth, so on the ground they show up as
  radial traces fanning out from the sensor - the pattern the real
  device draws. Mounted inverted here (-35..+5 deg), as in the vendor's
  demo footage; DEMO_MOUNT=upright flips it back.

  The driver is asked for interlace ${INTERLACE_H}x${INTERLACE_V}, so it
  accumulates that many scans into each published cloud: the density
  multiplies, and so does the time the cloud takes to build, which is
  why moving objects smear.

  Inject a fault and watch the cloud stop for 5 s, in another shell.
  Keep the spec on one line in YAML flow style: the ros2 CLI flattens
  multi-line service arguments, which the emulator then rejects.
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
    ros2 service call /hils_yvt35lx_emulator/inject_fault \\
      hils_bridge_interfaces/srv/InjectFault \\
      "{fault_yaml: '{fault_type: drop, target: data, duration_sec: 5.0, parameters: {probability: 1.0}}'}"

  Logs: $WORK    Stop: Ctrl-C
  ───────────────────────────────────────────────────────────────────

BANNER

# ── 4. RViz, or just idle when headless ──
if [ "$WITH_RVIZ" = "0" ]; then
    echo '[demo] headless (DEMO_RVIZ=0); Ctrl-C to stop'
    # Wait on a background job, never a foreground one: bash defers a
    # trapped signal until the foreground child returns, so `sleep 3600`
    # here would keep Ctrl-C waiting for up to an hour.
    while true; do sleep 3600 & wait $!; done
fi

cat > "$WORK/demo.rviz" <<'RVIZ'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /hokuyo_cloud21
      Splitter Ratio: 0.5
    Tree Height: 600
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 24
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: hokuyo_cloud2
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 4
      Size (m): 0.05
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /hokuyo_cloud2
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz_default_plugins/Axes
      Enabled: true
      Length: 1
      Name: sensor origin
      Radius: 0.05
      Reference Frame: <Fixed Frame>
      Value: true
  Enabled: true
  Global Options:
    Background Color: 32; 32; 40
    Fixed Frame: hokuyo3d
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 22
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 2
        Y: 0
        Z: -2
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.45
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 3.14
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 900
  Hide Left Dock: false
  Hide Right Dock: true
  Width: 1440
  X: 60
  Y: 60
RVIZ

echo '[demo] starting RViz (close it, or press Ctrl-C, to stop the demo)'
# Background + wait, so a Ctrl-C reaches the trap immediately instead of
# sitting behind RViz as a foreground child.
rviz2 -d "$WORK/demo.rviz" > "$WORK/rviz.log" 2>&1 &
RVIZ_PID=$!
PIDS+=($RVIZ_PID)
wait "$RVIZ_PID"
echo '[demo] RViz closed'
