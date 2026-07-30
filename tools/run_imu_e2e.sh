#!/bin/bash
# HILS end-to-end regression: WT901 checksum fault vs the real
# witmotion_ros driver over a serial pair.
#
#   Imu publisher -> hils_imu_bridge -> serial pair
#       -> witmotion_ros -> /imu  (observed by scenario_oracle)
#
# Requires (in the current shell): a sourced ROS 2 environment with
# hils_bridge_base / hils_bridge_imu_witmotion_wt901 / hils_bringup and
# witmotion_ros built, plus socat installed.
#
# Real-hardware mode (handover section 3.1): set E2E_SERIAL_BRIDGE and
# E2E_SERIAL_DRIVER to a physically cross-connected serial pair (e.g.
# /dev/ttyUSB0 and /dev/ttyUSB1) to run the same regression through the
# real USB-serial OS stack instead of a socat pty pair.
#
# Exit code: the oracle's verdict (0 = all expectations pass).
set -u

# See run_gps_e2e.sh: unicast-localhost discovery works everywhere this
# single-machine test runs, including CI containers without multicast.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${E2E_DISCOVERY_RANGE:-LOCALHOST}"

WORK=$(mktemp -d)
OBSERVE_DOMAIN="${ROS_DOMAIN_ID:-0}"
SCENARIO="$(ros2 pkg prefix hils_bringup)/share/hils_bringup/scenarios/imu/wt901_checksum_error_001.yaml"
BAUD=115200
PIDS=()

cleanup() {
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
}
trap cleanup EXIT

dump_logs() {
    echo '[e2e] ---- component logs ----'
    for f in socat imu_pub imu_bridge witmotion oracle runner; do
        if [ -f "$WORK/$f.log" ]; then
            echo "[e2e] == $f.log =="
            tail -n 30 "$WORK/$f.log"
        fi
    done
}

echo "[e2e] workdir: $WORK (domain $OBSERVE_DOMAIN," \
     "rmw ${RMW_IMPLEMENTATION:-default}," \
     "discovery $ROS_AUTOMATIC_DISCOVERY_RANGE)"

# 1. Serial pair: real cross-connected ports if given, socat pty otherwise
if [ -n "${E2E_SERIAL_BRIDGE:-}" ] && [ -n "${E2E_SERIAL_DRIVER:-}" ]; then
    ln -s "$E2E_SERIAL_BRIDGE" "$WORK/ttyBRIDGE"
    ln -s "$E2E_SERIAL_DRIVER" "$WORK/ttyDRIVER"
    echo "[e2e] real serial: bridge=$E2E_SERIAL_BRIDGE driver=$E2E_SERIAL_DRIVER"
else
    socat -d pty,raw,echo=0,link="$WORK/ttyBRIDGE" \
          pty,raw,echo=0,link="$WORK/ttyDRIVER" > "$WORK/socat.log" 2>&1 &
    PIDS+=($!)
    for _ in $(seq 50); do
        [ -e "$WORK/ttyBRIDGE" ] && [ -e "$WORK/ttyDRIVER" ] && break
        sleep 0.1
    done
    [ -e "$WORK/ttyDRIVER" ] || { echo "[e2e] socat failed"; exit 2; }
fi

# 2. Simulation source: 50 Hz Imu
python3 - > "$WORK/imu_pub.log" 2>&1 <<'EOF' &
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

rclpy.init()
node = Node('fake_imu_source')
pub = node.create_publisher(Imu, '/imu/data', 10)

def tick():
    msg = Imu()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'imu'
    msg.linear_acceleration.z = 9.8
    msg.angular_velocity.z = 0.1
    msg.orientation.w = 1.0
    pub.publish(msg)

node.create_timer(0.02, tick)
rclpy.spin(node)
EOF
PIDS+=($!)

# 3. Emulator bridge and real driver
ros2 run hils_bridge_imu_witmotion_wt901 imu_bridge_node --ros-args \
    -p serial_port:="$WORK/ttyBRIDGE" -p baudrate:="$BAUD" -p max_hz:=50.0 \
    > "$WORK/imu_bridge.log" 2>&1 &
PIDS+=($!)
sleep 2   # let the bridge stream before the driver opens the port

# witmotion_ros takes its port from a params file; patch a copy of the
# stock wt901 config to point at our driver-side port. QSerialPort keeps
# absolute paths as-is, so pty links work too. The stock 150 ms data
# timeout races with our bridge startup (a real WT901 streams from
# power-on), so widen it; it only fires on total silence, which the
# checksum fault never causes.
WT901_CFG="$WORK/wt901.yml"
cp "$(ros2 pkg prefix witmotion_ros)/share/witmotion_ros/config/wt901.yml" \
   "$WT901_CFG"
sed -i "s|port: .*|port: $WORK/ttyDRIVER|; s|baud_rate: .*|baud_rate: $BAUD|; \
        s|timeout_ms: .*|timeout_ms: 2000|" "$WT901_CFG"
ros2 run witmotion_ros witmotion_ros_node --ros-args \
    --params-file "$WT901_CFG" > "$WORK/witmotion.log" 2>&1 &
PIDS+=($!)

# Wait until the real driver republishes /imu (direct rclpy probe; see
# run_gps_e2e.sh for why ros2cli echo is avoided here).
python3 - <<'EOF' || {
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

rclpy.init()
node = Node('hils_e2e_probe')
got = []
node.create_subscription(Imu, '/imu', lambda _m: got.append(1), 10)
end = time.monotonic() + 60.0
while time.monotonic() < end and not got:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
EOF
    echo '[e2e] FAIL: /imu never appeared within 60s'
    dump_logs
    exit 2
}
echo '[e2e] normal path up, starting oracle + runner'

# 4. Oracle (judges) and runner (injects)
ros2 run hils_bridge_base scenario_oracle --ros-args \
    -p scenario_file:="$SCENARIO" \
    -p observe_domain_id:="$OBSERVE_DOMAIN" \
    -p output_dir:="$WORK/reports" \
    > "$WORK/oracle.log" 2>&1 &
ORACLE_PID=$!
PIDS+=($ORACLE_PID)
sleep 3
ros2 run hils_bridge_base scenario_runner --ros-args \
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
