#!/bin/bash
# Hardware-free HILS end-to-end regression: Hokuyo YVT-35LX VSSP stream
# faults vs the real urg3d_node2 driver over loopback TCP.
#
#   point source -> yvt35lx emulator -> VSSP TCP 127.0.0.1:10940
#       -> urg3d_node2 (lifecycle) -> /hokuyo_cloud2 + /imu
#       (observed by scenario_oracle)
#
# Default scenario (yvt35lx_blackout_001): the measurement stream goes
# silent for 3 s — under urg3d_node2's error_limit — and must resume
# without a reconnect or a crash. Pick another with
# E2E_SCENARIO=<name> (e.g. yvt35lx_stream_fault_001, the byte
# corruption case; see the package README for its known outcome).
#
# No simulator needed: the point source is a synthetic 10 m wall
# (default) or a PointCloud2 rosbag via E2E_BAG=<bag dir> (topic
# remapped from E2E_BAG_TOPIC, default /livox/lidar).
#
# Requires: built ws with hils_bridge_base / hils_bridge_interfaces /
# hils_bridge_lidar_hokuyo_yvt35lx / hils_bringup, plus urg3d_node2
# built from source (git clone --recursive
# https://github.com/Hokuyo-aut/urg3d_node2, needs ros-<distro>-laser-proc).
#
# Exit code: the oracle's verdict (0 = all expectations pass).
set -u

export ROS_AUTOMATIC_DISCOVERY_RANGE="${E2E_DISCOVERY_RANGE:-LOCALHOST}"

WORK=$(mktemp -d)
OBSERVE_DOMAIN="${ROS_DOMAIN_ID:-0}"
SCENARIO_NAME="${E2E_SCENARIO:-yvt35lx_blackout_001}"
SCENARIO="$(ros2 pkg prefix hils_bringup)/share/hils_bringup/scenarios/lidar/${SCENARIO_NAME}.yaml"
PIDS=()

if ! ros2 pkg prefix urg3d_node2 >/dev/null 2>&1; then
    echo '[e2e] FAIL: urg3d_node2 is not built in this workspace.'
    echo '       git clone https://github.com/Hokuyo-aut/urg3d_node2 (with'
    echo '       --recurse-submodules) into the colcon ws and rebuild.'
    exit 3
fi

cleanup() {
    # setsid gives each component its own process group; kill the whole
    # group so ros2 run/launch wrappers cannot leave orphan nodes that
    # confuse the next E2E run (stale scenario_runner services).
    for pid in "${PIDS[@]}"; do
        kill -- -"$pid" 2>/dev/null || kill "$pid" 2>/dev/null
    done
    pkill -f "[u]rg3d_node2_node" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT

dump_logs() {
    echo '[e2e] ---- component logs ----'
    for f in source emulator driver oracle runner; do
        if [ -f "$WORK/$f.log" ]; then
            echo "[e2e] == $f.log =="
            tail -n 30 "$WORK/$f.log"
        fi
    done
}

echo "[e2e] workdir: $WORK (domain $OBSERVE_DOMAIN," \
     "rmw ${RMW_IMPLEMENTATION:-default}," \
     "discovery $ROS_AUTOMATIC_DISCOVERY_RANGE)"

# 1. Simulation point source
if [ -n "${E2E_BAG:-}" ]; then
    echo "[e2e] point source: bag $E2E_BAG"
    setsid ros2 bag play "$E2E_BAG" --loop \
        --remap "${E2E_BAG_TOPIC:-/livox/lidar}:=/sim_points" \
        > "$WORK/source.log" 2>&1 &
    PIDS+=($!)
else
    echo '[e2e] point source: synthetic 10 m wall'
    setsid python3 - > "$WORK/source.log" 2>&1 <<'EOF' &
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

rclpy.init()
node = Node('fake_wall_source')
pub = node.create_publisher(PointCloud2, '/sim_points', 5)

pts = []
y = -3.0
while y <= 3.0:
    z = -1.5
    while z <= 1.5:
        pts.append((10.0, y, z, 100.0))
        z += 0.1
    y += 0.1
data = b''.join(struct.pack('<ffff', *p) for p in pts)

msg = PointCloud2()
msg.header.frame_id = 'lidar'
msg.height = 1
msg.width = len(pts)
msg.fields = [
    PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
    for n, o in (('x', 0), ('y', 4), ('z', 8), ('intensity', 12))]
msg.is_bigendian = False
msg.point_step = 16
msg.row_step = 16 * len(pts)
msg.is_dense = True
msg.data = data

def tick():
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)

node.create_timer(0.1, tick)
rclpy.spin(node)
EOF
    PIDS+=($!)
fi

# 2. Emulator (loopback) and real driver
setsid ros2 run hils_bridge_lidar_hokuyo_yvt35lx yvt35lx_emulator_node \
    --ros-args \
    -p device_ip:=127.0.0.1 -p host_ip:=127.0.0.1 \
    -p pointcloud_topic:=/sim_points \
    > "$WORK/emulator.log" 2>&1 &
PIDS+=($!)
sleep 3

# urg3d_node2 is a lifecycle node: run it, then drive configure/activate.
setsid ros2 run urg3d_node2 urg3d_node2_node --ros-args \
    -p ip_address:=127.0.0.1 -p ip_port:=10940 \
    -p publish_intensity:=true -p publish_auxiliary:=true \
    > "$WORK/driver.log" 2>&1 &
PIDS+=($!)
sleep 3

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
        case "$state" in
            "$want"*) OK=1; break ;;
        esac
        sleep 2
    done
    if [ "$OK" -ne 1 ]; then
        echo "[e2e] FAIL: lifecycle $transition did not reach $want (state: $state)"
        dump_logs
        exit 2
    fi
done

# Wait until the real driver publishes /hokuyo_cloud2 (RELIABLE QoS)
python3 - <<'EOF' || {
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('hils_e2e_probe')
got = []
node.create_subscription(PointCloud2, '/hokuyo_cloud2',
                         lambda _m: got.append(1), 10)
end = time.monotonic() + 60.0
while time.monotonic() < end and not got:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
EOF
    echo '[e2e] FAIL: /hokuyo_cloud2 never appeared within 60s'
    dump_logs
    exit 2
}
echo '[e2e] normal path up, starting oracle + runner'

# 3. Oracle (judges) and runner (injects)
setsid ros2 run hils_bridge_base scenario_oracle --ros-args \
    -p scenario_file:="$SCENARIO" \
    -p observe_domain_id:="$OBSERVE_DOMAIN" \
    -p output_dir:="$WORK/reports" \
    > "$WORK/oracle.log" 2>&1 &
ORACLE_PID=$!
PIDS+=($ORACLE_PID)
sleep 3
setsid ros2 run hils_bridge_base scenario_runner --ros-args \
    -p scenario_file:="$SCENARIO" > "$WORK/runner.log" 2>&1 &
PIDS+=($!)

wait "$ORACLE_PID"
CODE=$?

echo '[e2e] oracle verdicts:'
grep -aE '\[oracle\]' "$WORK/oracle.log" || cat "$WORK/oracle.log"
[ "$CODE" -eq 0 ] || dump_logs
cp -r "$WORK/reports" "${E2E_REPORT_DIR:-$WORK}" 2>/dev/null || true
echo "[e2e] reports: ${E2E_REPORT_DIR:-$WORK/reports}"
echo "[e2e] exit: $CODE"
exit "$CODE"
