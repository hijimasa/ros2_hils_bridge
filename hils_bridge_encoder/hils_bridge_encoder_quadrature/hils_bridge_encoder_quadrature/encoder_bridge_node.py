#!/usr/bin/env python3
"""
HILS Quadrature Encoder Bridge Node

Subscribes to a ROS JointState topic from a simulator, converts joint
positions to quadrature encoder counts, and sends them to an RP2040
(running the rp2040_encoder_quadrature firmware) via USB CDC using the
HILS framing protocol. The RP2040 emits A/B phase pulses on its GPIOs,
emulating the feedback signal from a real motor's encoder so the real PC's
controller can read it as if a real motor were attached.

Data flow:
    /joint_states -> angle-to-count conversion -> frame protocol -> serial -> RP2040
    RP2040 -> PIO A/B quadrature pulses -> real PC's encoder reader
"""

import math
import struct

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState

from hils_bridge_base.serial_bridge_base import SerialBridgeBase
from hils_bridge_base import frame_protocol

# Message type for encoder commands (must match firmware)
MSG_TYPE_ENCODER_CMD = 0x40


class EncoderBridgeNode(SerialBridgeBase):
    """ROS2 node that converts JointState messages to quadrature encoder commands."""

    def __init__(self):
        super().__init__(
            node_name='hils_encoder_quadrature_bridge',
            default_baudrate=115200,
        )

        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('encoder_channels', 2,
            ParameterDescriptor(description='Number of encoder channels (1-2)'))
        self.declare_parameter('encoder_cpr', 1000,
            ParameterDescriptor(description='Encoder counts per revolution'))
        self.declare_parameter('joint_names', [''],
            ParameterDescriptor(
                description='Ordered list of joint names to map to channels. '
                            'Empty string means use positional index.'))

        topic = self.get_parameter('joint_state_topic').value
        self.create_subscription(
            JointState, topic, self._joint_state_callback, 1)

        self.get_logger().info(
            f'Quadrature Encoder Bridge started: topic={topic}, '
            f'channels={self.get_parameter("encoder_channels").value}, '
            f'cpr={self.get_parameter("encoder_cpr").value}')

    def _angle_to_encoder_count(self, angle_rad: float) -> int:
        """Convert joint angle (radians) to encoder count: count = angle * cpr / (2 * pi)."""
        cpr = self.get_parameter('encoder_cpr').value
        return int(angle_rad * cpr / (2.0 * math.pi) + 0.5)

    def _find_joint_index(self, msg: JointState, joint_name: str) -> int:
        try:
            return list(msg.name).index(joint_name)
        except ValueError:
            return -1

    def _joint_state_callback(self, msg: JointState):
        if not self.check_rate_limit():
            return

        num_channels = self.get_parameter('encoder_channels').value
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
            count = self._angle_to_encoder_count(angle)
            channels.append((ch, count))

        if not channels:
            return

        # Header: msg_type (1) + channel_count (1)
        # Per channel: channel (1) + target_count (4, signed LE)
        payload = struct.pack('<BB', MSG_TYPE_ENCODER_CMD, len(channels))
        for ch, count in channels:
            payload += struct.pack('<Bi', ch, count)

        frame = frame_protocol.build_frame(payload)
        self.serial_write(frame)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
