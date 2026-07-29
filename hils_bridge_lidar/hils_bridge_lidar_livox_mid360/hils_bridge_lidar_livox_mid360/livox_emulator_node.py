#!/usr/bin/env python3
"""
Pure Software Livox Mid360 Emulator Node

Emulates a Livox Mid360 LiDAR entirely in software using a network interface
(e.g. USB-Ethernet adapter) assigned the LiDAR's IP address.
No W5500-EVB-Pico2 or custom firmware needed.

Implements the full Livox SDK2 protocol:
  - Discovery: responds to LidarSearch broadcasts on UDP 56000
  - Command: handles WorkModeControl, GetInternalInfo, etc. on UDP 56100
  - PointCloud: sends LivoxLidarEthernetPacket on UDP 56300 -> host:56301
  - IMU: sends IMU data on UDP 56400 -> host:56401

Setup:
  # Assign LiDAR IP to a USB-Ethernet adapter
  sudo ip addr add 192.168.1.12/24 dev eth1

  # Launch
  ros2 launch hils_bridge_lidar_livox_mid360 livox_emulator.launch.py

Data flow:
  Simulation (PointCloud2/IMU) -> this node -> UDP (Livox SDK2) -> Robot PC
"""

import struct
import threading
import time

import numpy as np

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2

from hils_bridge_base.device_state import state as device_states
from hils_bridge_base.udp_emulator_base import UdpEmulatorBase

# States in which the lidar has lost its work mode: after any of these
# the driver must send WorkModeControl again before streaming resumes
# (docs section 7.8: reboot requires reconfiguration by the driver).
_WORK_MODE_RESET_STATES = (
    device_states.POWER_OFF,
    device_states.BOOTING,
    device_states.REBOOTING,
    device_states.INTERNAL_ERROR,
)

# ── Livox SDK2 constants ──

LIVOX_SOF = 0xAA
LIVOX_DEV_TYPE_MID360 = 9

LIVOX_PORT_DISCOVERY = 56000
LIVOX_PORT_CMD_LIDAR = 56100
LIVOX_PORT_POINTCLOUD_LIDAR = 56300
LIVOX_PORT_POINTCLOUD_HOST = 56301
LIVOX_PORT_IMU_LIDAR = 56400
LIVOX_PORT_IMU_HOST = 56401

LIVOX_CMD_LIDAR_SEARCH = 0x0000
LIVOX_CMD_WORK_MODE_CTRL = 0x0100
LIVOX_CMD_GET_INTERNAL_INFO = 0x0101

LIVOX_WORK_MODE_NORMAL = 0x01

# Point cloud packet header size (without data[1] flexible member)
LIVOX_POINT_HEADER_SIZE = 36
# Max UDP payload within Ethernet MTU
LIVOX_MAX_UDP_PAYLOAD = 1472
# Max point data per packet
LIVOX_MAX_POINT_DATA = LIVOX_MAX_UDP_PAYLOAD - LIVOX_POINT_HEADER_SIZE

POINT_SIZE_HIGH = 14  # int32 x,y,z + uint8 refl + uint8 tag
POINT_SIZE_LOW = 8    # int16 x,y,z + uint8 refl + uint8 tag


# ── CRC functions ──

def _crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc


def _crc32(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


# ── Livox SDK2 packet builders ──

def _build_sdk2_packet(cmd_id: int, cmd_type: int, seq_num: int,
                       data: bytes = b'') -> bytes:
    """Build a Livox SDK2 command/response packet (24-byte header + data)."""
    total_len = 24 + len(data)
    # Build header without CRC fields first
    hdr = bytearray(24)
    hdr[0] = LIVOX_SOF                                 # sof
    hdr[1] = 0                                         # version
    struct.pack_into('<H', hdr, 2, total_len)          # length
    struct.pack_into('<I', hdr, 4, seq_num)            # seq_num
    struct.pack_into('<H', hdr, 8, cmd_id)             # cmd_id
    hdr[10] = cmd_type                                 # cmd_type (1=ACK)
    hdr[11] = 1                                        # sender_type (1=lidar)
    # rsvd[6] at offset 12 = zeros
    # CRC-16 over first 18 bytes
    struct.pack_into('<H', hdr, 18, _crc16_ccitt(bytes(hdr[:18])))
    # CRC-32 over data
    struct.pack_into('<I', hdr, 20, _crc32(data) if data else 0)
    return bytes(hdr) + data


def _build_point_packet(dot_num: int, data_type: int, timestamp_ns: int,
                        point_data: bytes, udp_cnt: int, frame_cnt: int) -> bytes:
    """Build a LivoxLidarEthernetPacket for point cloud data."""
    payload_len = len(point_data)
    hdr = bytearray(LIVOX_POINT_HEADER_SIZE)
    hdr[0] = 5                                         # version
    struct.pack_into('<H', hdr, 1, LIVOX_POINT_HEADER_SIZE + payload_len)  # length
    struct.pack_into('<H', hdr, 3, 47)                 # time_interval (0.1us units)
    struct.pack_into('<H', hdr, 5, dot_num)            # dot_num
    struct.pack_into('<H', hdr, 7, udp_cnt & 0xFFFF)   # udp_cnt
    hdr[9] = frame_cnt & 0xFF                          # frame_cnt
    hdr[10] = data_type                                # data_type
    hdr[11] = 0                                        # time_type (no sync)
    # rsvd[12] at offset 12 = zeros
    # crc32 at offset 24
    struct.pack_into('<I', hdr, 24, _crc32(point_data))
    # timestamp at offset 28
    struct.pack_into('<Q', hdr, 28, timestamp_ns)
    return bytes(hdr) + point_data


def _build_imu_packet(gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
                      timestamp_ns: int, udp_cnt: int, frame_cnt: int) -> bytes:
    """Build a LivoxLidarEthernetPacket for IMU data."""
    imu_data = struct.pack('<ffffff', gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z)
    hdr = bytearray(LIVOX_POINT_HEADER_SIZE)
    hdr[0] = 5
    struct.pack_into('<H', hdr, 1, LIVOX_POINT_HEADER_SIZE + len(imu_data))
    struct.pack_into('<H', hdr, 3, 0)                  # time_interval
    struct.pack_into('<H', hdr, 5, 1)                  # dot_num
    struct.pack_into('<H', hdr, 7, udp_cnt & 0xFFFF)
    hdr[9] = frame_cnt & 0xFF
    hdr[10] = 0                                        # data_type = IMU
    hdr[11] = 0
    struct.pack_into('<I', hdr, 24, _crc32(imu_data))
    struct.pack_into('<Q', hdr, 28, timestamp_ns)
    return bytes(hdr) + imu_data


# ── ROS Node ──

class LivoxEmulatorNode(UdpEmulatorBase):
    def __init__(self):
        super().__init__(
            node_name='hils_livox_emulator',
            default_device_ip='192.168.1.12',
            default_host_ip='192.168.1.5',
            # After a reboot the device is discoverable but not
            # streaming: the driver must re-send WorkModeControl.
            default_reboot_target=device_states.DISCOVERABLE,
        )
        self.device_state.add_listener(self._on_device_state_change)

        # Parameters (device_ip, host_ip, network_interface and max_hz
        # are declared by UdpEmulatorBase)
        self.declare_parameter('pointcloud_topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('max_points_per_frame', 20000,
            ParameterDescriptor(
                description='Max points per frame (no USB limit in direct UDP mode)',
                integer_range=[IntegerRange(from_value=100, to_value=100000, step=100)]))
        self.declare_parameter('point_data_type', 1,
            ParameterDescriptor(
                description='1=CartesianHigh(14B/pt), 2=CartesianLow(8B/pt)',
                integer_range=[IntegerRange(from_value=1, to_value=2, step=1)]))
        self.declare_parameter('downsample_mode', 'uniform',
            ParameterDescriptor(
                description='Downsample mode: "uniform" (even spacing) or "near" (closest points first)'))
        self.declare_parameter('serial_number', '0TFDFH600100511')
        self.declare_parameter('reboot_config_policy', 'preserve',
            ParameterDescriptor(
                description='Work mode across a reboot (docs 7.8). '
                            '"preserve": like the real MID-360, the saved '
                            'work mode is restored after boot and streaming '
                            'resumes without host interaction. "reset": the '
                            'work mode is lost and the driver must re-send '
                            'WorkModeControl - note Livox SDK2 never '
                            'reconfigures an already-known device, so with '
                            '"reset" the stream stays down until the driver '
                            'is restarted.'))

        self.add_on_set_parameters_callback(self._on_livox_param_change)

        self._serial_number = self.get_parameter('serial_number').value

        # State
        self._streaming = False
        self._was_streaming = False
        self._frame_cnt = 0
        self._points_sent = 0

        # Create UDP sockets via base class
        # Discovery uses 0.0.0.0 to receive broadcasts (255.255.255.255:56000)
        self._sock_discovery = self.create_device_socket(
            LIVOX_PORT_DISCOVERY, reuse=True, bind_any=True)
        self._sock_cmd = self.create_device_socket(LIVOX_PORT_CMD_LIDAR)
        self._sock_pointcloud = self.create_device_socket(
            LIVOX_PORT_POINTCLOUD_LIDAR)
        self._sock_imu = self.create_device_socket(LIVOX_PORT_IMU_LIDAR)

        self.get_logger().info(
            f'UDP sockets bound to {self.device_ip} '
            f'(discovery:{LIVOX_PORT_DISCOVERY}, cmd:{LIVOX_PORT_CMD_LIDAR}, '
            f'pc:{LIVOX_PORT_POINTCLOUD_LIDAR}, imu:{LIVOX_PORT_IMU_LIDAR})')

        # Start protocol handler thread
        self._quit = False
        self._protocol_thread = threading.Thread(target=self._protocol_loop, daemon=True)
        self._protocol_thread.start()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscriptions
        pc_topic = self.get_parameter('pointcloud_topic').value
        self.create_subscription(PointCloud2, pc_topic,
                                 self._pointcloud_callback, sensor_qos)
        if self.get_parameter('enable_imu').value:
            imu_topic = self.get_parameter('imu_topic').value
            self.create_subscription(Imu, imu_topic,
                                     self._imu_callback, sensor_qos)

        self.frame_count = 0

        self.get_logger().info(
            f'Livox Emulator started: topic={pc_topic}, '
            f'max_hz={self.get_parameter("max_hz").value}, '
            f'max_pts={self.get_parameter("max_points_per_frame").value}, '
            f'downsample={self.get_parameter("downsample_mode").value}')

    def _on_livox_param_change(self, params):
        for param in params:
            if param.name in ('max_points_per_frame', 'point_data_type',
                              'downsample_mode'):
                self.get_logger().info(f'{param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

    # ── Device state integration ──

    def _on_device_state_change(self, old_state, new_state):
        if new_state in _WORK_MODE_RESET_STATES:
            if self._streaming:
                self._was_streaming = True
                self._streaming = False
                self.get_logger().info(
                    f'Work mode suspended by state change {old_state} -> '
                    f'{new_state}')
            return
        if old_state == device_states.REBOOTING:
            # Boot complete. The real MID-360 persists its work mode and
            # resumes streaming on its own; Livox SDK2 never reconfigures
            # a known device, so "preserve" is required for the driver to
            # recover without a restart (docs 7.8, 22.4).
            policy = self.get_parameter('reboot_config_policy').value
            if policy == 'preserve' and self._was_streaming:
                self._streaming = True
                self._was_streaming = False
                self.get_logger().info(
                    'Boot complete: persisted work mode restored, '
                    'streaming resumes')
                self.device_state.set_state(device_states.STREAMING)
            else:
                self._was_streaming = False
                self.get_logger().info(
                    'Boot complete: work mode reset, waiting for '
                    'WorkModeControl from the driver')

    # ── Livox SDK2 protocol handler ──

    def _protocol_loop(self):
        """Background thread: handle discovery and command requests."""
        while not self._quit:
            self._poll_discovery()
            self._poll_command()
            time.sleep(0.001)

    def _poll_discovery(self):
        try:
            data, addr = self._sock_discovery.recvfrom(512)
        except BlockingIOError:
            return
        # A powered-off / rebooting device does not parse requests.
        if not self.device_state.is_open('discovery'):
            return
        if len(data) < 24 or data[0] != LIVOX_SOF:
            return
        cmd_id = struct.unpack_from('<H', data, 8)[0]
        if cmd_id != LIVOX_CMD_LIDAR_SEARCH:
            return
        cmd_type = data[10]
        if cmd_type != 0:  # not a request
            return
        seq_num = struct.unpack_from('<I', data, 4)[0]

        self.get_logger().info(f'Discovery from {addr}')

        # Build DetectionData response
        det = bytearray(24)
        det[0] = 0                           # ret_code = success
        det[1] = LIVOX_DEV_TYPE_MID360       # dev_type
        sn_bytes = self._serial_number.encode('ascii')[:16].ljust(16, b'\x00')
        det[2:18] = sn_bytes                 # sn[16]
        ip_parts = [int(x) for x in self.device_ip.split('.')]
        det[18:22] = bytes(ip_parts)         # lidar_ip[4]
        struct.pack_into('<H', det, 22, LIVOX_PORT_CMD_LIDAR)  # cmd_port

        resp = _build_sdk2_packet(LIVOX_CMD_LIDAR_SEARCH, 1, seq_num, bytes(det))
        self.send_udp(self._sock_discovery, resp, addr, channel='discovery')

    def _poll_command(self):
        try:
            data, addr = self._sock_cmd.recvfrom(512)
        except BlockingIOError:
            return
        # A powered-off / rebooting device does not parse requests.
        if not self.device_state.is_open('command'):
            return
        if len(data) < 24 or data[0] != LIVOX_SOF:
            return
        cmd_id = struct.unpack_from('<H', data, 8)[0]
        cmd_type = data[10]
        if cmd_type != 0:  # not a request
            return
        seq_num = struct.unpack_from('<I', data, 4)[0]

        if cmd_id == LIVOX_CMD_WORK_MODE_CTRL:
            if len(data) >= 25:
                mode = data[24]
                self._streaming = (mode == LIVOX_WORK_MODE_NORMAL)
                self.get_logger().info(
                    f'WorkMode={mode}, streaming={self._streaming}')
                # Reflect the work mode in the device state so the data
                # and imu channels open/close accordingly.
                self.device_state.set_state(
                    device_states.STREAMING if self._streaming
                    else device_states.READY)
            resp = _build_sdk2_packet(cmd_id, 1, seq_num, b'\x00')

        elif cmd_id == LIVOX_CMD_GET_INTERNAL_INFO:
            # Minimal: ret_code=0, param_num=0
            resp = _build_sdk2_packet(cmd_id, 1, seq_num, b'\x00\x00\x00')

        else:
            self.get_logger().debug(f'Unknown cmd_id=0x{cmd_id:04X}')
            resp = _build_sdk2_packet(cmd_id, 1, seq_num, b'\x00')

        self.send_udp(self._sock_cmd, resp, addr, channel='command')

    # ── Point cloud processing ──

    def _pointcloud_callback(self, msg: PointCloud2):
        if not self._streaming:
            return

        if not self.check_rate_limit():
            return

        data_type = self.get_parameter('point_data_type').value
        max_pts = self.get_parameter('max_points_per_frame').value
        downsample_mode = self.get_parameter('downsample_mode').value

        try:
            result = self._extract_points(msg, max_pts, data_type, downsample_mode)
        except Exception as e:
            self.get_logger().warn(f'Point extraction failed: {e}')
            return

        if result is None:
            return

        point_data, dot_num = result
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Determine point size and split into MTU-compliant packets
        point_size = POINT_SIZE_HIGH if data_type == 1 else POINT_SIZE_LOW
        max_pts_per_pkt = LIVOX_MAX_POINT_DATA // point_size

        offset = 0
        remaining = dot_num
        host = (self.host_ip, LIVOX_PORT_POINTCLOUD_HOST)

        while remaining > 0:
            chunk = min(remaining, max_pts_per_pkt)
            chunk_bytes = point_data[offset:offset + chunk * point_size]
            pkt = _build_point_packet(chunk, data_type, timestamp_ns,
                                      chunk_bytes, self.next_udp_cnt(),
                                      self._frame_cnt)
            if not self.send_udp(self._sock_pointcloud, pkt, host,
                                 channel='data'):
                return

            self._points_sent += chunk
            offset += chunk * point_size
            remaining -= chunk
            timestamp_ns += int(chunk * 4.7e3)

        self._frame_cnt += 1
        self.mark_sent()
        self.frame_count += 1

    def _extract_points(self, msg: PointCloud2, max_pts: int, data_type: int,
                        downsample_mode: str):
        """Extract and pack points using numpy vectorized operations."""
        field_names = ['x', 'y', 'z']
        has_intensity = any(f.name in ('intensity', 'reflectivity')
                           for f in msg.fields)
        if has_intensity:
            int_field = 'intensity' if any(
                f.name == 'intensity' for f in msg.fields) else 'reflectivity'
            field_names.append(int_field)

        pts_arr = point_cloud2.read_points(msg, field_names=field_names,
                                           skip_nans=True)
        if pts_arr is None or len(pts_arr) == 0:
            return None

        n_pts = len(pts_arr)

        # Downsample if needed
        if n_pts > max_pts:
            if downsample_mode == 'near':
                # Keep closest points (by distance from origin)
                dist_sq = (pts_arr['x'].astype(np.float64) ** 2 +
                           pts_arr['y'].astype(np.float64) ** 2 +
                           pts_arr['z'].astype(np.float64) ** 2)
                indices = np.argpartition(dist_sq, max_pts)[:max_pts]
                pts_arr = pts_arr[indices]
            else:  # uniform
                indices = np.linspace(0, n_pts - 1, max_pts, dtype=np.intp)
                pts_arr = pts_arr[indices]
            n_pts = max_pts

        # Reflectivity
        if has_intensity:
            refl = np.clip(pts_arr[int_field], 0, 255).astype(np.uint8)
        else:
            refl = np.zeros(n_pts, dtype=np.uint8)
        tag = np.zeros(n_pts, dtype=np.uint8)

        if data_type == 1:
            x = (pts_arr['x'] * 1000.0).astype(np.int32)
            y = (pts_arr['y'] * 1000.0).astype(np.int32)
            z = (pts_arr['z'] * 1000.0).astype(np.int32)
            dt = np.dtype([('x', '<i4'), ('y', '<i4'), ('z', '<i4'),
                           ('r', 'u1'), ('t', 'u1')])
        else:
            x = np.clip((pts_arr['x'] * 100.0).astype(np.int32),
                        -32768, 32767).astype(np.int16)
            y = np.clip((pts_arr['y'] * 100.0).astype(np.int32),
                        -32768, 32767).astype(np.int16)
            z = np.clip((pts_arr['z'] * 100.0).astype(np.int32),
                        -32768, 32767).astype(np.int16)
            dt = np.dtype([('x', '<i2'), ('y', '<i2'), ('z', '<i2'),
                           ('r', 'u1'), ('t', 'u1')])

        arr = np.empty(n_pts, dtype=dt)
        arr['x'] = x
        arr['y'] = y
        arr['z'] = z
        arr['r'] = refl
        arr['t'] = tag
        return arr.tobytes(), n_pts

    # ── IMU ──

    def _imu_callback(self, msg: Imu):
        if not self._streaming:
            return
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        pkt = _build_imu_packet(
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
            timestamp_ns, self.next_udp_cnt(), self._frame_cnt)
        self.send_udp(self._sock_imu, pkt,
                      (self.host_ip, LIVOX_PORT_IMU_HOST), channel='imu')

    # ── Stats ──

    def _stats_callback(self):
        state = 'streaming' if self._streaming else 'waiting'
        self.get_logger().info(
            f'Status: {state}, points_sent={self._points_sent}, '
            f'frames={self.frame_count}')

    # ── Cleanup ──

    def destroy_node(self):
        self._quit = True
        if hasattr(self, '_protocol_thread'):
            self._protocol_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LivoxEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
