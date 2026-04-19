#!/usr/bin/env python3
"""
HILS RC Servo PWM Bridge Node

Subscribes to a ROS JointState topic from a simulator, converts joint
positions to servo pulse widths, and sends them to an RP2040 (running
the rp2040_actuator_servo_pwm firmware) via USB CDC using the HILS
framing protocol.

Encoder feedback emulation lives in the separate
hils_bridge_encoder_quadrature package, since PWM (controller -> servo)
and encoder pulses (motor -> controller) are different roles in the
HILS loop and should not share a payload.

Data flow:
    /joint_states -> angle-to-pulse conversion -> frame protocol -> serial -> RP2040
    RP2040 -> PIO PWM servo signals
"""

import struct
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from sensor_msgs.msg import JointState

from hils_bridge_base.serial_bridge_base import SerialBridgeBase
from hils_bridge_base import frame_protocol

# Message type for servo commands (must match firmware)
MSG_TYPE_SERVO_CMD = 0x20


class PwmBridgeNode(SerialBridgeBase):
    """ROS2 node that converts JointState messages to servo PWM commands."""

    def __init__(self):
        super().__init__(
            node_name='hils_servo_pwm_bridge',
            default_baudrate=115200,
        )

        # Declare parameters
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('servo_channels', 4,
            ParameterDescriptor(description='Number of servo channels (1-4)'))
        self.declare_parameter('servo_min_angle', -math.pi / 2,
            ParameterDescriptor(
                description='Minimum servo angle in radians',
                floating_point_range=[FloatingPointRange(
                    from_value=-math.pi, to_value=0.0, step=0.0)]))
        self.declare_parameter('servo_max_angle', math.pi / 2,
            ParameterDescriptor(
                description='Maximum servo angle in radians',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, to_value=math.pi, step=0.0)]))
        self.declare_parameter('servo_min_pulse_us', 500,
            ParameterDescriptor(description='Minimum servo pulse width (us)'))
        self.declare_parameter('servo_max_pulse_us', 2500,
            ParameterDescriptor(description='Maximum servo pulse width (us)'))
        self.declare_parameter('joint_names', [''],
            ParameterDescriptor(
                description='Ordered list of joint names to map to channels. '
                            'Empty string means use positional index.'))

        # Subscribe to JointState
        topic = self.get_parameter('joint_state_topic').value
        self.create_subscription(
            JointState, topic, self._joint_state_callback, 1)

        self.get_logger().info(
            f'Servo PWM Bridge started: topic={topic}, '
            f'channels={self.get_parameter("servo_channels").value}, '
            f'pulse=[{self.get_parameter("servo_min_pulse_us").value}, '
            f'{self.get_parameter("servo_max_pulse_us").value}] us, '
            f'angle=[{self.get_parameter("servo_min_angle").value:.3f}, '
            f'{self.get_parameter("servo_max_angle").value:.3f}] rad')

    def _angle_to_pulse_us(self, angle_rad: float) -> int:
        """Linearly map [servo_min_angle, servo_max_angle] to
        [servo_min_pulse_us, servo_max_pulse_us]; clamp to [500, 2500] us.
        """
        min_angle = self.get_parameter('servo_min_angle').value
        max_angle = self.get_parameter('servo_max_angle').value
        min_pulse = self.get_parameter('servo_min_pulse_us').value
        max_pulse = self.get_parameter('servo_max_pulse_us').value

        angle_rad = max(min_angle, min(max_angle, angle_rad))
        t = (angle_rad - min_angle) / (max_angle - min_angle)
        pulse = int(min_pulse + t * (max_pulse - min_pulse) + 0.5)
        return max(500, min(2500, pulse))

    def _find_joint_index(self, msg: JointState, joint_name: str) -> int:
        try:
            return list(msg.name).index(joint_name)
        except ValueError:
            return -1

    def _joint_state_callback(self, msg: JointState):
        """Convert JointState to servo commands and send via serial."""
        if not self.check_rate_limit():
            return

        num_channels = self.get_parameter('servo_channels').value
        joint_names = self.get_parameter('joint_names').value

        channels = []
        for ch in range(num_channels):
            if joint_names and ch < len(joint_names) and joint_names[ch]:
                joint_idx = self._find_joint_index(msg, joint_names[ch])
                if joint_idx < 0:
                    continue
            else:
                joint_idx = ch

            if joint_idx >= len(msg.position):
                continue

            angle = msg.position[joint_idx]
            pulse_us = self._angle_to_pulse_us(angle)
            channels.append((ch, pulse_us))

        if not channels:
            return

        # Header: msg_type (1) + channel_count (1)
        # Per channel: channel (1) + pulse_us (2, LE)
        payload = struct.pack('<BB', MSG_TYPE_SERVO_CMD, len(channels))
        for ch, pulse in channels:
            payload += struct.pack('<BH', ch, pulse)

        frame = frame_protocol.build_frame(payload)
        self.serial_write(frame)


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
