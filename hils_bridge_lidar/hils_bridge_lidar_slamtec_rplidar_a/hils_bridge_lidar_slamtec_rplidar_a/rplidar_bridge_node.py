#!/usr/bin/env python3
"""
HILS SLAMTEC RPLIDAR A-series Bridge Node

Emulates a SLAMTEC RPLIDAR (A-series serial protocol) so the real
host-side driver (rplidar_ros / the SLAMTEC SDK) can be exercised
without hardware: the driver opens the serial port and issues
requests; this node answers and, after a SCAN request, streams the
subscribed sensor_msgs/LaserScan as 5-byte measurement nodes.

Python port of REACT-simulator's rplidar_emulator
(rplidar_emulator_node.cpp / rplidar_protocol.cpp), rebuilt on
SerialBridgeBase so the standard HILS fault pipeline (drop / corrupt /
delay / freeze / duplicate ...) and device state machine apply to
every reply.

Data flow:
    Driver (host) --0xA5 requests--> FT234X or PTY --> this node
        GET_INFO / GET_HEALTH / GET_SAMPLERATE /
        GET_LIDAR_CONF               -> descriptor + reply ('command')
        SCAN / FORCE_SCAN / EXPRESS / HQ -> descriptor ('command'),
                                            then continuous stream
        STOP / RESET                  -> stream stops, no reply
    Simulator --LaserScan on scan_topic--> this node
        ranges/intensities -> 5-byte standard measurement nodes,
        one revolution per 100 ms while scanning ('scan' channel)

Like the C++ emulator: EXPRESS/HQ scans are acknowledged with the
matching descriptor but the stream falls back to standard 5-byte
nodes; when no LaserScan has arrived, a synthetic 360-point pattern
is streamed so the driver still sees data.
"""

import math
import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import LaserScan

from hils_bridge_base.serial_bridge_base import SerialBridgeBase

from hils_bridge_lidar_slamtec_rplidar_a import rplidar_protocol as proto

# The C++ emulator streams one revolution every 100 ms (10 Hz).
SCAN_PERIOD_SEC = 0.1
# Warn when the scan topic has been silent this long while scanning.
NO_DATA_WARN_SEC = 1.0


class RplidarBridgeNode(SerialBridgeBase):
    """ROS2 node that answers SLAMTEC RPLIDAR requests as the sensor."""

    def __init__(self):
        super().__init__(
            node_name='hils_rplidar_bridge',
            default_baudrate=1000000,   # matches the C++ configure()
            default_serial_port='/tmp/rplidar_front',
            frame_protocol_firmware=False,  # raw vendor wire protocol
        )

        self.declare_parameter('scan_topic', '/scan',
            ParameterDescriptor(
                description='LaserScan topic from the simulator, '
                            'streamed as scan measurements.'))
        # Device info reported by GET_DEVICE_INFO (C++ defaults).
        self.declare_parameter('model_code', 0x61,
            ParameterDescriptor(description='Model byte (0x61 = S2).'))
        self.declare_parameter('firmware_version', 0x0118,
            ParameterDescriptor(
                description='16-bit firmware version (0x0118 = v1.24).'))
        self.declare_parameter('hardware_version', 0x07,
            ParameterDescriptor(description='Hardware version byte.'))
        self.declare_parameter('serial_number', 'EMULATOR00000001',
            ParameterDescriptor(
                description='16-byte serial number (padded/truncated).'))

        self._device_info = proto.DeviceInfo(
            model=self.get_parameter('model_code').value,
            firmware_version=self.get_parameter('firmware_version').value,
            hardware_version=self.get_parameter('hardware_version').value,
            serial_number=self.get_parameter(
                'serial_number').value.encode('ascii', errors='replace'))

        # Scan state. _scanning flips on SCAN-family requests and off
        # on STOP / RESET; the stream runs only while it is set.
        self._scanning = False
        self._current_scan = []
        self._scan_lock = threading.Lock()
        self._last_scan_recv_time = time.monotonic()
        self._warned_no_data = False
        self._dummy_angle_offset = 0.0
        self._revolutions_sent = 0

        # ROS interface (simulator-facing)
        scan_topic = self.get_parameter('scan_topic').value
        self.create_subscription(
            LaserScan, scan_topic, self._scan_callback, 10)

        # Serial side (driver-facing): background RX + stream thread.
        self._rx_buffer = bytearray()
        self._stop_event = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._serial_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            f'RPLIDAR bridge started: scan_topic={scan_topic}, '
            f'model=0x{self._device_info.model:02X}, '
            f'serial={self.get_parameter("serial_number").value}')

    # ---------- Serial input (driver side) ----------

    def _serial_loop(self):
        """Background thread: parse requests and pace the scan stream."""
        last_scan_time = time.monotonic()
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                data = self._serial.read(4096)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read failed: {e}')
                return

            if data:
                self._rx_buffer.extend(data)
                while True:
                    parsed = proto.extract_command(self._rx_buffer)
                    if parsed is None:
                        break
                    self._handle_command(*parsed)

            if self._scanning:
                now = time.monotonic()
                if now - last_scan_time >= SCAN_PERIOD_SEC:
                    self._send_scan_data()
                    last_scan_time = now

            if not data:
                time.sleep(0.005)

    def _handle_command(self, cmd: int, payload: bytes):
        """Handle one request from the driver."""
        # Device state gate on the REQUEST side: a powered-off /
        # rebooting device does not parse requests. The request is
        # consumed but nothing is answered -- the line goes silent,
        # which is exactly what the driver sees on a dead sensor.
        if not self.device_state.is_open('command'):
            return

        if cmd == proto.CMD_GET_DEVICE_INFO:
            self.serial_write(
                proto.build_device_info_response(self._device_info),
                channel='command')

        elif cmd == proto.CMD_GET_DEVICE_HEALTH:
            self.serial_write(
                proto.build_health_response(proto.STATUS_OK, 0),
                channel='command')

        elif cmd == proto.CMD_GET_SAMPLERATE:
            self.serial_write(
                proto.build_samplerate_response(),
                channel='command')

        elif cmd == proto.CMD_GET_LIDAR_CONF:
            parsed = proto.parse_get_lidar_conf_payload(payload)
            if parsed is None:
                self.get_logger().warning(
                    f'GET_LIDAR_CONF payload too short '
                    f'({len(payload)} bytes)')
                return
            conf_type, mode_id = parsed
            response = proto.build_lidar_conf_response(conf_type, mode_id)
            if response is None:
                self.get_logger().warning(
                    f'Unknown LIDAR_CONF type: 0x{conf_type:08X}')
                return
            self.serial_write(response, channel='command')

        elif cmd in (proto.CMD_SCAN, proto.CMD_FORCE_SCAN):
            self.serial_write(
                proto.build_scan_descriptor(proto.ANS_TYPE_MEASUREMENT),
                channel='command')
            self._scanning = True
            self.get_logger().info(f'SCAN start (cmd=0x{cmd:02X})')

        elif cmd in (proto.CMD_EXPRESS_SCAN, proto.CMD_HQ_SCAN):
            # Acknowledged with the matching descriptor, but the
            # stream falls back to standard nodes (like the C++).
            if cmd == proto.CMD_EXPRESS_SCAN:
                working_mode = payload[0] if payload else 0
                ans_type = (proto.ANS_TYPE_MEASUREMENT if working_mode == 0
                            else proto.ANS_TYPE_MEASUREMENT_CAPSULED)
            else:
                ans_type = proto.ANS_TYPE_MEASUREMENT_HQ
            self.serial_write(proto.build_scan_descriptor(ans_type),
                              channel='command')
            self._scanning = True
            self.get_logger().info(
                f'EXPRESS/HQ SCAN start (cmd=0x{cmd:02X}, '
                f'descriptor type=0x{ans_type:02X}, stream=standard nodes)')

        elif cmd == proto.CMD_STOP:
            self._scanning = False
            self.get_logger().info('STOP: scan stream stopped')

        elif cmd == proto.CMD_RESET:
            self._scanning = False
            self.get_logger().info('RESET: scan stream stopped')

        elif cmd in (proto.CMD_SET_MOTOR_PWM, proto.CMD_HQ_MOTOR_SPEED):
            self.get_logger().debug(
                f'Motor control command 0x{cmd:02X} (ignored)')

        else:
            self.get_logger().warning(f'Unknown command: 0x{cmd:02X}')

    # ---------- Scan stream (driver side) ----------

    def _send_scan_data(self):
        """Send one revolution of 5-byte measurement nodes.

        No check_rate_limit(): the SCAN request opened a continuous
        stream and the sensor sets its own 10 Hz cadence; suppression
        is the job of the fault pipeline / device state, not the
        rate limiter.
        """
        with self._scan_lock:
            scan_copy = list(self._current_scan)
            elapsed = time.monotonic() - self._last_scan_recv_time
            if elapsed > NO_DATA_WARN_SEC:
                if not self._warned_no_data:
                    self.get_logger().warning(
                        f'No scan data received from topic '
                        f'"{self.get_parameter("scan_topic").value}" for '
                        f'{elapsed:.1f} s. Is the simulator running?')
                    self._warned_no_data = True
            else:
                self._warned_no_data = False

        # All-invalid data would make the SDK's ascendScanData fail
        # (every distance zero): fall back to the dummy pattern.
        if scan_copy and not any(
                p.valid and p.distance > 0 for p in scan_copy):
            self.get_logger().warning(
                'All scan points invalid, using dummy data',
                throttle_duration_sec=5.0)
            scan_copy = []

        if not scan_copy:
            scan_copy = proto.dummy_scan(self._dummy_angle_offset)
            self._dummy_angle_offset += 0.5
            if self._dummy_angle_offset >= 360.0:
                self._dummy_angle_offset = 0.0

        self.serial_write(proto.encode_scan(scan_copy), channel='scan')
        self._revolutions_sent += 1

    # ---------- Feedback (simulator side) ----------

    def _scan_callback(self, msg: LaserScan):
        """Store the latest LaserScan as wire-ready scan points."""
        points = []
        angle = msg.angle_min
        n_intensities = len(msg.intensities)
        for i, r in enumerate(msg.ranges):
            # Strict less-than on range_max: simulators report
            # range_max for "no hit" rays.
            valid = math.isfinite(r) and \
                msg.range_min <= r < msg.range_max
            if i < n_intensities:
                quality = int(min(255.0, msg.intensities[i]))
            else:
                quality = 47 if valid else 0
            points.append(proto.ScanPoint(
                angle=angle, distance=r, quality=quality, valid=valid))
            angle += msg.angle_increment

        with self._scan_lock:
            self._current_scan = points
            self._last_scan_recv_time = time.monotonic()

    # ---------- Housekeeping ----------

    def _stats_callback(self):
        self.get_logger().info(
            f'Status: scanning={self._scanning}, '
            f'revolutions={self._revolutions_sent}, '
            f'port={self.get_parameter("serial_port").value}')

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, '_rx_thread') and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RplidarBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
