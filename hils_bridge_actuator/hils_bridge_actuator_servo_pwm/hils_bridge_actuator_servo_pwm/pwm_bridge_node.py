#!/usr/bin/env python3
"""
HILS RC Servo PWM Capture Bridge Node

Reads measured servo PWM pulse widths reported by the RP2040
(rp2040_actuator_servo_pwm firmware) via USB CDC and publishes them
as ROS topics so the simulation / test harness can evaluate the PWM
commands the robot controller is issuing.

Data flow:
    Robot controller PWM -> RP2040 GPIO (PIO capture) -> USB CDC
                         -> this node -> ROS topics
                            - sensor_msgs/JointState (converted angle)
                            - std_msgs/UInt16MultiArray (raw pulse widths, us)

Encoder output emulation (feedback direction, motor -> controller)
remains in the separate hils_bridge_encoder_quadrature package, since
PWM input (controller -> servo) and encoder pulses play different
roles in the HILS loop and should not share a payload.
"""

import math
import struct
import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, UInt16MultiArray

from hils_bridge_base import frame_protocol

# Message type emitted by firmware (must match HILS_MSG_TYPE_SERVO_MEASURED).
MSG_TYPE_SERVO_MEASURED = 0x21

# One header (msg_type, channel_count) + N entries (channel, valid, pulse_us, period_us).
_HEADER_STRUCT = struct.Struct('<BB')
_CHANNEL_STRUCT = struct.Struct('<BBHH')


class PwmBridgeNode(Node):
    """ROS2 node that reads PWM pulse measurements from RP2040 and publishes them."""

    def __init__(self):
        super().__init__('hils_servo_pwm_bridge')

        # Serial parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        # Output topics
        self.declare_parameter('joint_state_topic', '/servo_pwm/joint_states')
        self.declare_parameter('pulse_topic', '/servo_pwm/pulses_us')

        # Channel configuration
        self.declare_parameter('servo_channels', 4,
            ParameterDescriptor(description='Number of servo channels to publish (1-4)'))
        self.declare_parameter('joint_names', [''],
            ParameterDescriptor(
                description='Ordered list of joint names per channel. '
                            'Empty string means "servo_<channel>".'))

        # Pulse <-> angle mapping (must match the controller under test).
        self.declare_parameter('servo_min_angle', -math.pi / 2,
            ParameterDescriptor(
                description='Angle mapped to servo_min_pulse_us (rad)',
                floating_point_range=[FloatingPointRange(
                    from_value=-math.pi, to_value=0.0, step=0.0)]))
        self.declare_parameter('servo_max_angle', math.pi / 2,
            ParameterDescriptor(
                description='Angle mapped to servo_max_pulse_us (rad)',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, to_value=math.pi, step=0.0)]))
        self.declare_parameter('servo_min_pulse_us', 500,
            ParameterDescriptor(description='Pulse width corresponding to servo_min_angle (us)'))
        self.declare_parameter('servo_max_pulse_us', 2500,
            ParameterDescriptor(description='Pulse width corresponding to servo_max_angle (us)'))

        # Publishers
        self._joint_pub = self.create_publisher(
            JointState, self.get_parameter('joint_state_topic').value, 10)
        self._pulse_pub = self.create_publisher(
            UInt16MultiArray, self.get_parameter('pulse_topic').value, 10)

        # Open serial port
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        try:
            self._serial = serial.Serial(port, baudrate, timeout=0)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise
        self.get_logger().info(f'Opened serial port: {port} @ {baudrate} baud')

        # Frame receiver and background read thread
        self._receiver = frame_protocol.FrameProtocolReceiver()
        self._stop_event = threading.Event()
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        # Stats
        self._report_count = 0
        self.create_timer(10.0, self._stats_callback)

        self.get_logger().info(
            f'Servo PWM capture bridge started: '
            f'joint_state_topic={self.get_parameter("joint_state_topic").value}, '
            f'pulse_topic={self.get_parameter("pulse_topic").value}, '
            f'channels={self.get_parameter("servo_channels").value}, '
            f'pulse=[{self.get_parameter("servo_min_pulse_us").value}, '
            f'{self.get_parameter("servo_max_pulse_us").value}] us, '
            f'angle=[{self.get_parameter("servo_min_angle").value:.3f}, '
            f'{self.get_parameter("servo_max_angle").value:.3f}] rad')

    # ---------- Serial input ----------

    def _serial_read_loop(self):
        """Background thread: drain the serial port and parse frames."""
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                data = self._serial.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read failed: {e}')
                return

            if not data:
                time.sleep(0.005)
                continue

            for payload in self._receiver.feed(data):
                self._handle_payload(payload)

    def _handle_payload(self, payload: bytes) -> None:
        if len(payload) < _HEADER_STRUCT.size:
            return

        msg_type, channel_count = _HEADER_STRUCT.unpack_from(payload, 0)
        if msg_type != MSG_TYPE_SERVO_MEASURED:
            return

        expected = _HEADER_STRUCT.size + channel_count * _CHANNEL_STRUCT.size
        if len(payload) < expected:
            self.get_logger().warn(
                f'Truncated servo measurement: got {len(payload)} bytes, '
                f'expected {expected}')
            return

        channels = []
        offset = _HEADER_STRUCT.size
        for _ in range(channel_count):
            channels.append(_CHANNEL_STRUCT.unpack_from(payload, offset))
            offset += _CHANNEL_STRUCT.size

        self._publish_measurements(channels)
        self._report_count += 1

    # ---------- Publishing ----------

    def _publish_measurements(self, channels) -> None:
        """Publish measured pulses as JointState and raw UInt16MultiArray."""
        num = min(self.get_parameter('servo_channels').value, len(channels))
        if num <= 0:
            return

        joint_names_param = self.get_parameter('joint_names').value
        names = []
        positions = []
        pulses = []

        now = self.get_clock().now().to_msg()

        for idx in range(num):
            ch, valid, pulse_us, _period_us = channels[idx]

            if joint_names_param and ch < len(joint_names_param) and joint_names_param[ch]:
                names.append(joint_names_param[ch])
            else:
                names.append(f'servo_{ch}')

            pulses.append(pulse_us if valid else 0)

            if valid:
                positions.append(self._pulse_to_angle_rad(pulse_us))
            else:
                positions.append(float('nan'))

        js = JointState()
        js.header = Header(stamp=now, frame_id='')
        js.name = names
        js.position = positions
        self._joint_pub.publish(js)

        arr = UInt16MultiArray()
        arr.data = pulses
        self._pulse_pub.publish(arr)

    def _pulse_to_angle_rad(self, pulse_us: int) -> float:
        """Inverse of the angle -> pulse mapping used on the controller side."""
        min_angle = self.get_parameter('servo_min_angle').value
        max_angle = self.get_parameter('servo_max_angle').value
        min_pulse = self.get_parameter('servo_min_pulse_us').value
        max_pulse = self.get_parameter('servo_max_pulse_us').value

        if max_pulse == min_pulse:
            return min_angle
        t = (pulse_us - min_pulse) / (max_pulse - min_pulse)
        return min_angle + t * (max_angle - min_angle)

    # ---------- Housekeeping ----------

    def _stats_callback(self):
        self.get_logger().info(
            f'Status: reports={self._report_count}, '
            f'port={self.get_parameter("serial_port").value}')

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, '_serial') and self._serial.is_open:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PwmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
