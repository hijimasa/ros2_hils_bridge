#!/usr/bin/env python3
"""
HILS UVC Bridge Node

Subscribes to a ROS image topic, JPEG-encodes the frames, and sends them
to Pico#1 via USB CDC serial using the HILS framing protocol.

Receives resolution change commands from the UVC host (via Pico#2 -> Pico#1)
and dynamically adjusts the output resolution.

Data flow:
    /image_raw -> JPEG encode -> frame protocol -> serial -> Pico#1 -> UART -> Pico#2 -> UVC
    UVC host -> Pico#2 -> UART -> Pico#1 -> serial -> this node (resolution update)

JPEG quality and resolution can be changed at runtime:
    ros2 param set /hils_uvc_bridge jpeg_quality 80
"""

import io

import cv2
import serial
import time
import threading

from PIL import Image as PilImage

import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from hils_bridge_base import frame_protocol
from hils_bridge_base.serial_bridge_base import SerialBridgeBase


class UvcBridgeNode(SerialBridgeBase):
    def __init__(self):
        # serial_port, baudrate (ignored by USB CDC), max_hz and the fault
        # injection services come from SerialBridgeBase.
        super().__init__(
            'hils_uvc_bridge',
            default_serial_port='/dev/ttyACM0',
            default_max_hz=15.0,
        )

        # Parameters with descriptors for discoverability
        self.declare_parameter('jpeg_quality', 50,
            ParameterDescriptor(
                description='JPEG encoding quality (1-100). Higher = better quality, larger frames.',
                integer_range=[IntegerRange(from_value=1, to_value=100, step=1)]))
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        # Parameter change callback
        self.add_on_set_parameters_callback(self._on_uvc_param_change)

        # Subscribe to image topic
        topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(
            Image, topic, self.image_callback, 1)  # queue_size=1: drop old frames

        self.bridge = CvBridge()
        self.frame_count = 0

        # Start reverse-channel read thread
        self._receiver = frame_protocol.FrameProtocolReceiver()
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info(
            f'UVC Bridge started: topic={topic}, quality={self.get_parameter("jpeg_quality").value}, '
            f'max_hz={self.get_parameter("max_hz").value}')

    def _on_uvc_param_change(self, params):
        for param in params:
            if param.name in ('jpeg_quality', 'frame_width', 'frame_height'):
                self.get_logger().info(f'{param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

    def _serial_read_loop(self):
        """Background thread: reads reverse-channel commands from Pico#1."""
        while rclpy.ok():
            try:
                data = self._serial.read(256)
                if data:
                    for payload in self._receiver.feed(data):
                        self._handle_command(payload)
                else:
                    time.sleep(0.01)
            except (serial.SerialException, OSError):
                break

    def _handle_command(self, payload: bytes):
        """Process a reverse-channel command."""
        result = frame_protocol.parse_resolution_cmd(payload)
        if result:
            width, height, frame_index = result
            self.get_logger().info(
                f'UVC host selected resolution: {width}x{height} (index={frame_index})')
            self.set_parameters([
                Parameter('frame_width', Parameter.Type.INTEGER, width),
                Parameter('frame_height', Parameter.Type.INTEGER, height),
            ])

    def image_callback(self, msg: Image):
        if not self.check_rate_limit():
            return

        # Convert ROS Image to OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        # Resize to match UVC descriptor resolution
        target_w = self.get_parameter('frame_width').value
        target_h = self.get_parameter('frame_height').value
        h, w = cv_image.shape[:2]
        if w != target_w or h != target_h:
            cv_image = cv2.resize(cv_image, (target_w, target_h))

        # JPEG encode with 4:2:2 chroma subsampling, like a real UVC
        # camera. Hosts assume this: usb_cam's mjpeg2rgb pre-builds its
        # swscale context for 4:2:2 and reads past the smaller chroma
        # planes of a 4:2:0 frame (segfault). cv2.imencode can only
        # produce 4:2:0 until OpenCV 4.7, hence PIL.
        quality = self.get_parameter('jpeg_quality').value
        try:
            jpeg_io = io.BytesIO()
            PilImage.fromarray(cv_image[:, :, ::-1]).save(
                jpeg_io, 'JPEG', quality=quality, subsampling=1)
        except (OSError, ValueError) as e:
            self.get_logger().warn(f'JPEG encode failed: {e}')
            return

        jpeg_bytes = jpeg_io.getvalue()

        # Build framed packet and send through the fault pipeline
        try:
            frame = frame_protocol.build_frame(jpeg_bytes)
        except ValueError as e:
            self.get_logger().error(f'Frame build failed: {e}')
            return
        if not self.serial_write(frame, channel='video'):
            return

        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Sent {self.frame_count} frames '
                f'(quality={quality}, {target_w}x{target_h}, '
                f'JPEG={len(jpeg_bytes)} bytes)')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UvcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
