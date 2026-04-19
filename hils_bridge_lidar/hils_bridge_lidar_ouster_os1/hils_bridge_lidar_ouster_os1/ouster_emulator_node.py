#!/usr/bin/env python3
"""
Pure Software Ouster OS1 Emulator Node (LEGACY UDP profile)

Emulates an Ouster OS1 LiDAR entirely in software using a network interface
(e.g. USB-Ethernet adapter) assigned the LiDAR's IP address.

Protocol coverage:
  - HTTP port 80: REST API (/api/v1/sensor/metadata, /api/v1/sensor/config, ...)
  - UDP 7502: LEGACY lidar packets (16 columns/packet, with 4B trailer)
  - UDP 7503: IMU packets (48B)

NOTE: HTTP port 80 requires special Docker config.
  docker-compose.yml must set: sysctls:
    - net.ipv4.ip_unprivileged_port_start=80

Setup:
  sudo ip addr add 192.168.1.100/24 dev eth1
  ros2 launch hils_bridge_lidar_ouster_os1 ouster_emulator.launch.py \
    network_interface:=eth1 host_ip:=192.168.1.5

CAUTION: Unlike Velodyne (where the pain was a byte-order typo in a magic
flag), Ouster compatibility depends on matching three things simultaneously:
  1. UDP packets must pack exactly 16 columns per datagram (LEGACY profile)
  2. Each column must include a 4-byte status trailer (0xFFFFFFFF = valid)
  3. TCP 7501 must respond to at least get_sensor_info / get_beam_intrinsics
The ouster_ros driver connects to TCP 7501 first to fetch metadata. Without
the TCP server, the driver never starts reading UDP.
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

from hils_bridge_base.udp_emulator_base import UdpEmulatorBase
from hils_bridge_lidar_ouster_os1 import ouster_protocol as op


class OusterEmulatorNode(UdpEmulatorBase):
    """ROS2 node that converts PointCloud2/Imu to Ouster OS1 LEGACY UDP packets."""

    def __init__(self):
        super().__init__(
            node_name='hils_ouster_emulator',
            default_device_ip='192.168.1.100',
            default_host_ip='192.168.1.5',
        )

        # ── Parameters ──
        self.declare_parameter('pointcloud_topic', '/ouster/points',
            ParameterDescriptor(description='PointCloud2 topic from simulation'))
        self.declare_parameter('imu_topic', '/ouster/imu',
            ParameterDescriptor(description='Imu topic from simulation'))
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('max_points_per_frame', 30000,
            ParameterDescriptor(
                description='Max points per frame',
                integer_range=[IntegerRange(
                    from_value=100, to_value=300000, step=100)]))
        self.declare_parameter('downsample_mode', 'uniform',
            ParameterDescriptor(
                description='Downsample mode: "uniform" or "near"'))
        self.declare_parameter('lidar_mode', '512x10',
            ParameterDescriptor(
                description='Lidar mode string, e.g. 512x10, 1024x10, 2048x10'))
        self.declare_parameter('n_channels', 16,
            ParameterDescriptor(
                description='Number of vertical channels (16, 32, or 64)',
                integer_range=[IntegerRange(
                    from_value=16, to_value=64, step=16)]))
        self.declare_parameter('serial_number', '992008000101',
            ParameterDescriptor(description='Emulated LiDAR serial number'))
        self.declare_parameter('elevation_filter', True,
            ParameterDescriptor(
                description='Drop points outside sensor vertical FOV'))

        self.add_on_set_parameters_callback(self._on_ouster_param_change)

        # ── Resolve channel count / lidar mode ──
        n_channels = self.get_parameter('n_channels').value
        lidar_mode = self.get_parameter('lidar_mode').value
        self._columns_per_frame = int(lidar_mode.split('x')[0])

        # ── Beam angles ──
        self._beam_angles_deg = np.array(
            op.default_beam_angles(n_channels), dtype=np.float64)

        # ── Build full metadata (mutable; updated via HTTP PUT) ──
        self._metadata = op.build_full_metadata(
            n_channels=n_channels,
            lidar_mode=lidar_mode,
            columns_per_frame=self._columns_per_frame,
            serial=self.get_parameter('serial_number').value,
            prod_pn='840-101855-02',
            fw_rev='v2.5.2',  # Must be >= 2.4 for modern ouster_ros driver
            udp_dest=self.host_ip,
            beam_angles=self._beam_angles_deg.tolist(),
        )

        # ── HTTP REST API server (port 80) ──
        try:
            self._http_server = op.HttpApiServer(
                bind_ip=self.device_ip,
                metadata_provider=lambda: self._metadata,
                on_config_change=self._on_http_config_change,
                logger=self.get_logger(),
            )
            self._http_server.start()
            self.get_logger().info(
                f'HTTP API listening on http://{self.device_ip}:{op.OUSTER_HTTP_PORT}/')
        except PermissionError as e:
            self.get_logger().error(
                f'Cannot bind HTTP port 80: {e}\n'
                f'  Add to docker-compose.yml under this service:\n'
                f'    sysctls:\n'
                f'      - net.ipv4.ip_unprivileged_port_start=80')
            raise

        # ── UDP sockets ──
        self._lidar_sock = self.create_device_socket(op.OUSTER_LIDAR_PORT)
        self._imu_sock = self.create_device_socket(op.OUSTER_IMU_PORT)

        self.get_logger().info(
            f'Ouster emulator sockets: HTTP {op.OUSTER_HTTP_PORT} (control), '
            f'UDP {op.OUSTER_LIDAR_PORT} (lidar), '
            f'UDP {op.OUSTER_IMU_PORT} (imu)')

        # ── State ──
        self._frame_count = 0
        self._points_sent = 0
        self._imu_count = 0

        # Dynamic UDP destination ports. The ouster_ros driver with
        # lidar_port:=0 binds random ports but still reports 7502/7503 in
        # set_config_param, so that mechanism is unreliable. Instead, we
        # *listen* on our sending sockets: if the driver sends any UDP
        # datagram to us (e.g., stray handshake, port-unreachable ICMP is
        # not visible, but some drivers do send keep-alive), we learn its
        # source port and retarget future packets there.
        self._dst_port_lidar = op.OUSTER_LIDAR_PORT
        self._dst_port_imu = op.OUSTER_IMU_PORT
        self._port_lock = threading.Lock()

        # Background thread listens on both sockets for incoming UDP
        self._listen_quit = False
        self._listen_thread = threading.Thread(
            target=self._listen_loop, daemon=True)
        self._listen_thread.start()

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # ── Subscriptions ──
        self.create_subscription(
            PointCloud2, self.get_parameter('pointcloud_topic').value,
            self._pointcloud_callback, sensor_qos)
        if self.get_parameter('enable_imu').value:
            self.create_subscription(
                Imu, self.get_parameter('imu_topic').value,
                self._imu_callback, sensor_qos)

        self.get_logger().info(
            f'Ouster OS1 emulator started: '
            f'mode={lidar_mode}, n_channels={n_channels}, '
            f'SN={self.get_parameter("serial_number").value}')

    # ── Parameter callback ──

    def _on_ouster_param_change(self, params):
        for param in params:
            if param.name in ('max_points_per_frame', 'max_hz',
                              'downsample_mode'):
                self.get_logger().info(f'{param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

    def _on_http_config_change(self, updates: dict):
        """Invoked when a client PUT /api/v1/sensor/config with updates."""
        self.get_logger().info(f'HTTP set config: {updates}')
        cfg = self._metadata.setdefault('config_params', {})
        cfg.update(updates)
        # Re-target UDP destination if the client sets udp_dest
        udp_dest = updates.get('udp_dest')
        if isinstance(udp_dest, str) and udp_dest:
            self._host_ip = udp_dest
            self.get_logger().info(f'UDP destination updated to {udp_dest}')
        # Update UDP destination ports (the driver usually sets these to its
        # own random-bound sockets)
        with self._port_lock:
            p_lidar = updates.get('udp_port_lidar')
            if isinstance(p_lidar, int) and p_lidar > 0:
                self._dst_port_lidar = p_lidar
                self.get_logger().info(
                    f'UDP lidar destination port updated to {p_lidar}')
            p_imu = updates.get('udp_port_imu')
            if isinstance(p_imu, int) and p_imu > 0:
                self._dst_port_imu = p_imu
                self.get_logger().info(
                    f'UDP imu destination port updated to {p_imu}')

    # ── UDP listen loop: auto-discover driver's actual bound port ──

    def _listen_loop(self):
        """Listen on lidar/imu sockets. If the driver sends any UDP to us,
        treat the source port as the driver's listening port and retarget.

        This is a fallback for when `set_config_param` reports incorrect
        port numbers (e.g. when ouster_ros is launched with lidar_port:=0).
        """
        while not self._listen_quit:
            for sock, label in ((self._lidar_sock, 'lidar'),
                                (self._imu_sock, 'imu')):
                try:
                    data, addr = sock.recvfrom(65535)
                except BlockingIOError:
                    continue
                except OSError:
                    continue
                src_ip, src_port = addr[0], addr[1]
                # Only trust packets from the configured host
                if src_ip != self._host_ip:
                    continue
                with self._port_lock:
                    if label == 'lidar' and self._dst_port_lidar != src_port:
                        old = self._dst_port_lidar
                        self._dst_port_lidar = src_port
                        self.get_logger().info(
                            f'[auto-discover] lidar dest port {old} -> '
                            f'{src_port} (from driver UDP at {src_ip})')
                    elif label == 'imu' and self._dst_port_imu != src_port:
                        old = self._dst_port_imu
                        self._dst_port_imu = src_port
                        self.get_logger().info(
                            f'[auto-discover] imu dest port {old} -> '
                            f'{src_port} (from driver UDP at {src_ip})')
            time.sleep(0.01)

    # ── Point cloud → LEGACY lidar packets ──

    def _pointcloud_callback(self, msg: PointCloud2):
        if not self.check_rate_limit():
            return

        try:
            packets = self._convert_to_lidar_packets(msg)
        except Exception as e:
            self.get_logger().warn(f'Point conversion failed: {e}')
            return

        if not packets:
            return

        host = (self.host_ip, self._dst_port_lidar)
        for pkt in packets:
            try:
                self._lidar_sock.sendto(pkt, host)
            except OSError as e:
                self.get_logger().error(f'UDP send failed: {e}')
                return

        self.mark_sent()
        self._frame_count += 1

    def _convert_to_lidar_packets(self, msg: PointCloud2) -> list:
        """Convert a PointCloud2 to a list of LEGACY Ouster lidar UDP packets."""
        max_pts = self.get_parameter('max_points_per_frame').value
        downsample_mode = self.get_parameter('downsample_mode').value
        n_channels = self.get_parameter('n_channels').value

        # Extract points
        field_names = ['x', 'y', 'z']
        has_intensity = any(f.name in ('intensity', 'reflectivity')
                           for f in msg.fields)
        if has_intensity:
            int_field = ('intensity' if any(f.name == 'intensity'
                         for f in msg.fields) else 'reflectivity')
            field_names.append(int_field)

        pts_arr = point_cloud2.read_points(
            msg, field_names=field_names, skip_nans=True)
        if pts_arr is None or len(pts_arr) == 0:
            return []

        n_pts = len(pts_arr)

        # Downsample
        if n_pts > max_pts:
            if downsample_mode == 'near':
                dist_sq = (pts_arr['x'].astype(np.float64) ** 2 +
                           pts_arr['y'].astype(np.float64) ** 2 +
                           pts_arr['z'].astype(np.float64) ** 2)
                indices = np.argpartition(dist_sq, max_pts)[:max_pts]
                pts_arr = pts_arr[indices]
            else:
                indices = np.linspace(0, n_pts - 1, max_pts, dtype=np.intp)
                pts_arr = pts_arr[indices]
            n_pts = max_pts

        x = pts_arr['x'].astype(np.float64)
        y = pts_arr['y'].astype(np.float64)
        z = pts_arr['z'].astype(np.float64)
        if has_intensity:
            refl = np.clip(pts_arr[int_field], 0, 65535).astype(np.uint16)
        else:
            refl = np.zeros(n_pts, dtype=np.uint16)

        # Spherical
        xy_dist = np.sqrt(x * x + y * y)
        distance = np.sqrt(x * x + y * y + z * z)
        azimuth_deg = np.degrees(np.arctan2(y, x)) % 360.0
        elevation_deg = np.degrees(np.arctan2(z, xy_dist))

        # Filter by vertical FOV if enabled
        if self.get_parameter('elevation_filter').value:
            beam_min = float(self._beam_angles_deg.min()) - 1.0
            beam_max = float(self._beam_angles_deg.max()) + 1.0
            mask = (elevation_deg >= beam_min) & (elevation_deg <= beam_max)
            if not np.any(mask):
                return []
            azimuth_deg = azimuth_deg[mask]
            elevation_deg = elevation_deg[mask]
            distance = distance[mask]
            refl = refl[mask]

        if len(distance) == 0:
            return []

        # Map to nearest beam channel
        elev_diff = np.abs(
            elevation_deg[:, np.newaxis] - self._beam_angles_deg[np.newaxis, :])
        channel_idx = np.argmin(elev_diff, axis=1).astype(np.int32)

        # Range (mm, uint32)
        range_mm = np.clip(distance * 1000.0, 0, 0xFFFFFFFF).astype(np.uint32)

        # Column index (0 .. columns_per_frame-1)
        cpf = self._columns_per_frame
        col_idx = ((azimuth_deg / 360.0) * cpf).astype(np.int32)
        col_idx = np.clip(col_idx, 0, cpf - 1)

        # Encoder count per column
        encoder_per_col = op.ENCODER_TICKS_PER_REV // cpf

        # Timestamp base
        base_ts_ns = (msg.header.stamp.sec * 1_000_000_000
                      + msg.header.stamp.nanosec)
        # Nominal scan duration = 100ms at 10Hz; per-column ns increment
        col_time_ns = 100_000_000 // cpf

        frame_id = self._frame_count & 0xFFFF

        # ── Build column blocks for every column index (0..cpf-1) ──
        # Column blocks missing measurements still get a valid header + empty
        # channel data + valid trailer so the scan is continuous.
        columns = []
        # Sort points by column
        sort_idx = np.argsort(col_idx)
        col_idx = col_idx[sort_idx]
        channel_idx = channel_idx[sort_idx]
        range_mm = range_mm[sort_idx]
        refl = refl[sort_idx]

        # Build a dict: col -> list of (ch, rng, refl)
        # Faster with np.split using boundaries
        boundaries = np.searchsorted(col_idx,
                                     np.arange(cpf + 1, dtype=np.int32))
        channel_bytes_size = n_channels * op.CHANNEL_DATA_SIZE

        for col in range(cpf):
            lo = boundaries[col]
            hi = boundaries[col + 1]
            n_in_col = hi - lo

            channel_data = bytearray(channel_bytes_size)
            if n_in_col > 0:
                c_ch = channel_idx[lo:hi]
                c_rng = range_mm[lo:hi]
                c_refl = refl[lo:hi]
                # For each measurement, write to its channel slot, keeping
                # the closest return when multiple points map to the same
                # channel within this column.
                for i in range(n_in_col):
                    ch = int(c_ch[i])
                    if ch < 0 or ch >= n_channels:
                        continue
                    offset = ch * op.CHANNEL_DATA_SIZE
                    existing_range = struct.unpack_from(
                        '<I', channel_data, offset)[0]
                    new_range = int(c_rng[i])
                    if new_range == 0:
                        continue
                    if existing_range == 0 or new_range < existing_range:
                        struct.pack_into('<I', channel_data, offset,
                                         new_range)
                        struct.pack_into('<H', channel_data, offset + 4,
                                         int(c_refl[i]))  # reflectivity
                        struct.pack_into('<H', channel_data, offset + 6,
                                         int(c_refl[i]))  # signal (reuse)
                        struct.pack_into('<H', channel_data, offset + 8,
                                         0)  # near_ir
                        struct.pack_into('<H', channel_data, offset + 10,
                                         0)  # unused

            col_ts = base_ts_ns + col * col_time_ns
            enc = (col * encoder_per_col) % op.ENCODER_TICKS_PER_REV
            block = op.build_column_block(
                timestamp_ns=col_ts,
                measurement_id=col,
                frame_id=frame_id,
                encoder_count=enc,
                channel_data=bytes(channel_data),
                status=op.COLUMN_STATUS_VALID,
            )
            columns.append(block)
            self._points_sent += int(n_in_col)

        # ── Pack COLUMNS_PER_PACKET columns into each UDP packet ──
        packets = []
        for i in range(0, len(columns), op.COLUMNS_PER_PACKET):
            chunk = columns[i:i + op.COLUMNS_PER_PACKET]
            if len(chunk) < op.COLUMNS_PER_PACKET:
                # Pad the last packet with duplicate of the last column
                # (driver expects fixed-size packets). Use empty channels
                # with valid header to avoid spurious returns.
                empty_channel = bytes(n_channels * op.CHANNEL_DATA_SIZE)
                while len(chunk) < op.COLUMNS_PER_PACKET:
                    pad_col = (i + len(chunk))
                    pad_block = op.build_column_block(
                        timestamp_ns=base_ts_ns + pad_col * col_time_ns,
                        measurement_id=pad_col,
                        frame_id=frame_id,
                        encoder_count=(pad_col * encoder_per_col)
                                      % op.ENCODER_TICKS_PER_REV,
                        channel_data=empty_channel,
                        status=op.COLUMN_STATUS_VALID,
                    )
                    chunk.append(pad_block)
            packets.append(b''.join(chunk))

        return packets

    # ── IMU ──

    def _imu_callback(self, msg: Imu):
        timestamp_ns = (msg.header.stamp.sec * 1_000_000_000
                        + msg.header.stamp.nanosec)
        pkt = op.build_imu_packet(
            timestamp_ns,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z)
        try:
            self._imu_sock.sendto(pkt, (self.host_ip, self._dst_port_imu))
            self._imu_count += 1
        except OSError as e:
            self.get_logger().debug(f'IMU send failed: {e}')

    # ── Stats ──

    def _stats_callback(self):
        with self._port_lock:
            dst_lidar = self._dst_port_lidar
            dst_imu = self._dst_port_imu
        self.get_logger().info(
            f'Status: frames={self._frame_count}, '
            f'points_sent={self._points_sent}, '
            f'imu_pkts={self._imu_count}, '
            f'dst={self.host_ip}:{dst_lidar}/{dst_imu} (lidar/imu)')

    # ── Cleanup ──

    def destroy_node(self):
        self._listen_quit = True
        try:
            self._http_server.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OusterEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
