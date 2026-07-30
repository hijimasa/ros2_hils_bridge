#!/bin/bash
# Hardware-free HILS end-to-end regression: OS1 HTTP config-API faults
# vs the real ouster_ros driver over loopback.
#
#   point source -> ouster emulator -> HTTP 127.0.0.1:80 + UDP
#       -> ouster_ros -> /ouster/points  (observed by scenario_oracle)
#
# The scenario degrades only the config API (HTTP 500 -> truncated JSON
# -> silence) and expects the already-running data stream to survive.
#
# Loopback notes: binding port 80 needs root or
# net.ipv4.ip_unprivileged_port_start=80; the emulator ignores its own
# echoed UDP packets so auto-discovery works in a single namespace.
#
# Point source: a synthetic 10 m wall (default, works in CI) or a
# PointCloud2 rosbag via E2E_BAG=<bag dir> (topic remapped from
# E2E_BAG_TOPIC, default /livox/lidar).
#
# Requires: built ws with hils_bridge_base / hils_bridge_lidar_ouster_os1 /
# hils_bringup, plus ros-<distro>-ouster-ros.
#
# Exit code: the oracle's verdict (0 = all expectations pass).
set -u

export ROS_AUTOMATIC_DISCOVERY_RANGE="${E2E_DISCOVERY_RANGE:-LOCALHOST}"

WORK=$(mktemp -d)
OBSERVE_DOMAIN="${ROS_DOMAIN_ID:-0}"
SCENARIO="$(ros2 pkg prefix hils_bringup)/share/hils_bringup/scenarios/lidar/ouster_http_error_001.yaml"
PIDS=()

cleanup() {
    # setsid gives each component its own process group; kill the whole
    # group so ros2 run/launch wrappers cannot leave orphan nodes that
    # confuse the next E2E run (stale scenario_runner services).
    for pid in "${PIDS[@]}"; do
        kill -- -"$pid" 2>/dev/null || kill "$pid" 2>/dev/null
    done
    # sensor.launch.xml spawns children the shell PID list misses
    pkill -f "[o]s_driver" 2>/dev/null
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
setsid ros2 run hils_bridge_lidar_ouster_os1 ouster_emulator_node --ros-args \
    -p device_ip:=127.0.0.1 -p host_ip:=127.0.0.1 \
    -p pointcloud_topic:=/sim_points \
    > "$WORK/emulator.log" 2>&1 &
PIDS+=($!)
sleep 3

setsid ros2 launch ouster_ros sensor.launch.xml \
    sensor_hostname:=127.0.0.1 viz:=false \
    > "$WORK/driver.log" 2>&1 &
PIDS+=($!)

# Wait until the real driver republishes /ouster/points (BEST_EFFORT)
python3 - <<'EOF' || {
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

rclpy.init()
node = Node('hils_e2e_probe')
got = []
qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
node.create_subscription(PointCloud2, '/ouster/points',
                         lambda _m: got.append(1), qos)
end = time.monotonic() + 90.0
while time.monotonic() < end and not got:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
EOF
    echo '[e2e] FAIL: /ouster/points never appeared within 90s'
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
