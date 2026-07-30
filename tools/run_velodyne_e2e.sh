#!/bin/bash
# Hardware-free HILS end-to-end regression: VLP-16 logical power loss
# vs the real velodyne driver over loopback UDP.
#
#   point source -> velodyne emulator -> UDP 127.0.0.1:2368
#       -> velodyne_driver -> velodyne_transform_node -> /velodyne_points
#       (observed by scenario_oracle)
#
# The emulator binds ephemeral source ports (bind_data_port:=0) so it
# can share the loopback namespace with the driver's listen socket -
# no second network namespace or container needed.
#
# Point source: a synthetic 10 m wall (default, works in CI) or a
# PointCloud2 rosbag via E2E_BAG=<bag dir> (topic remapped from
# E2E_BAG_TOPIC, default /livox/lidar).
#
# Requires: built ws with hils_bridge_base / hils_bridge_lidar_velodyne_vlp16 /
# hils_bringup, plus the velodyne driver stack (velodyne_driver +
# velodyne_pointcloud).
#
# Exit code: the oracle's verdict (0 = all expectations pass).
set -u

export ROS_AUTOMATIC_DISCOVERY_RANGE="${E2E_DISCOVERY_RANGE:-LOCALHOST}"

WORK=$(mktemp -d)
OBSERVE_DOMAIN="${ROS_DOMAIN_ID:-0}"
SCENARIO="$(ros2 pkg prefix hils_bringup)/share/hils_bringup/scenarios/lidar/velodyne_power_loss_001.yaml"
PIDS=()

cleanup() {
    # setsid gives each component its own process group; kill the whole
    # group so ros2 run/launch wrappers cannot leave orphan nodes that
    # confuse the next E2E run (stale scenario_runner services).
    for pid in "${PIDS[@]}"; do
        kill -- -"$pid" 2>/dev/null || kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
}
trap cleanup EXIT

dump_logs() {
    echo '[e2e] ---- component logs ----'
    for f in source emulator driver transform oracle runner; do
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
import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

rclpy.init()
node = Node('fake_wall_source')
pub = node.create_publisher(PointCloud2, '/sim_points', 5)

# 10 m wall: y in [-3, 3], z in [-1.5, 1.5] - inside the VLP-16 FOV
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

# 2. Emulator (loopback, ephemeral source ports) and real driver stack
setsid ros2 run hils_bridge_lidar_velodyne_vlp16 velodyne_emulator_node --ros-args \
    -p device_ip:=127.0.0.1 -p host_ip:=127.0.0.1 \
    -p bind_data_port:=0 -p bind_position_port:=0 \
    -p pointcloud_topic:=/sim_points \
    > "$WORK/emulator.log" 2>&1 &
PIDS+=($!)

setsid ros2 run velodyne_driver velodyne_driver_node --ros-args \
    -p device_ip:=127.0.0.1 -p model:=VLP16 -p port:=2368 -p rpm:=600.0 \
    -p gps_time:=false -p read_fast:=false -p read_once:=false \
    -p repeat_delay:=0.0 -p timestamp_first_packet:=false \
    > "$WORK/driver.log" 2>&1 &
PIDS+=($!)

setsid ros2 run velodyne_pointcloud velodyne_transform_node --ros-args \
    -p model:=VLP16 -p calibration:="$(ros2 pkg prefix velodyne_pointcloud)/share/velodyne_pointcloud/params/VLP16db.yaml" \
    -p min_range:=0.4 -p max_range:=130.0 \
    > "$WORK/transform.log" 2>&1 &
PIDS+=($!)

# Wait until the real driver stack republishes /velodyne_points
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
node.create_subscription(PointCloud2, '/velodyne_points',
                         lambda _m: got.append(1), qos)
end = time.monotonic() + 60.0
while time.monotonic() < end and not got:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
EOF
    echo '[e2e] FAIL: /velodyne_points never appeared within 60s'
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
