#!/bin/bash
# Hardware-free HILS end-to-end regression: NMEA checksum fault vs the
# real nmea_navsat_driver over a socat pty pair.
#
#   NavSatFix publisher -> hils_gps_bridge -> pty <-> pty
#       -> nmea_navsat_driver -> /fix  (observed by scenario_oracle)
#
# Requires (in the current shell): a sourced ROS 2 environment with
# hils_bridge_base / hils_bridge_gps_nmea0183 / hils_bringup built, plus
# ros-<distro>-nmea-navsat-driver and socat installed.
#
# Real-hardware mode (handover section 3.1): set E2E_SERIAL_BRIDGE and
# E2E_SERIAL_DRIVER to a physically cross-connected serial pair (e.g.
# /dev/ttyUSB0 and /dev/ttyUSB1) to run the same regression through the
# real USB-serial OS stack instead of a socat pty pair.
#
# Exit code: the oracle's verdict (0 = all expectations pass).
set -u

# CI containers (e.g. GitHub Actions) often have no working multicast,
# which breaks default DDS participant discovery. LOCALHOST restricts
# discovery to unicast localhost peers and works everywhere this
# single-machine test runs. The ROS setup files preset SUBNET, so this
# must be forced, not defaulted; override via E2E_DISCOVERY_RANGE.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${E2E_DISCOVERY_RANGE:-LOCALHOST}"

WORK=$(mktemp -d)
OBSERVE_DOMAIN="${ROS_DOMAIN_ID:-0}"
SCENARIO="$(ros2 pkg prefix hils_bringup)/share/hils_bringup/scenarios/gps/gps_checksum_error_001.yaml"
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
    for f in socat fix_pub gps_bridge nmea_driver oracle runner; do
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

# 2. Simulation source: 5 Hz NavSatFix
setsid python3 - > "$WORK/fix_pub.log" 2>&1 <<'EOF' &
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

rclpy.init()
node = Node('fake_gps_source')
pub = node.create_publisher(NavSatFix, '/gps/fix', 10)

def tick():
    msg = NavSatFix()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'gps'
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude, msg.longitude, msg.altitude = 35.6812, 139.7671, 40.0
    pub.publish(msg)

node.create_timer(0.2, tick)
rclpy.spin(node)
EOF
PIDS+=($!)

# 3. Emulator bridge and real driver
setsid ros2 run hils_bridge_gps_nmea0183 gps_bridge_node --ros-args \
    -p serial_port:="$WORK/ttyBRIDGE" -p baudrate:=9600 \
    > "$WORK/gps_bridge.log" 2>&1 &
PIDS+=($!)
setsid ros2 run nmea_navsat_driver nmea_serial_driver --ros-args \
    -p port:="$WORK/ttyDRIVER" -p baud:=9600 \
    > "$WORK/nmea_driver.log" 2>&1 &
PIDS+=($!)

# Wait until the real driver republishes /fix. Probe with a direct
# rclpy subscription: ros2cli's echo depends on the daemon for type
# resolution, which is unreliable in CI containers.
python3 - <<'EOF' || {
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

rclpy.init()
node = Node('hils_e2e_probe')
got = []
node.create_subscription(NavSatFix, '/fix', lambda _m: got.append(1), 10)
end = time.monotonic() + 60.0
while time.monotonic() < end and not got:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got else 1)
EOF
    echo '[e2e] FAIL: /fix never appeared within 60s'
    dump_logs
    exit 2
}
echo '[e2e] normal path up, starting oracle + runner'

# 4. Oracle (judges) and runner (injects)
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
