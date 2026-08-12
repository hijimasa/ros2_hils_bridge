#!/usr/bin/env python3
"""
Pure Software Hokuyo YVT-35LX (3D-URG) Emulator Node

Emulates the VSSP 2.1 TCP protocol of the YVT-35LX so the real
urg3d_node2 driver (github.com/Hokuyo-aut/urg3d_node2) can be
evaluated without hardware and without a simulator: point clouds from
any PointCloud2 source (synthetic wall, rosbag, or a live simulator)
are re-scanned into the YVT line/spot pattern and streamed as _ri/_ro
packets.

Protocol coverage (validated against urg3d_library, the driver's
protocol parser):
  - TCP 10940: VER / GET:stat / GET:_itl / GET:_itv / GET:tblh /
    GET:tvNN / SET:_itl / SET:_itv / DAT:ro|ri|ax / RST
  - _ri / _ro range packets (36 lines x 74 spots, VSSP 2.1 22-byte
    range header, full-line index blocks)
  - _ax auxiliary packets (gyro / accel / compass / temperature)

Interlace note: SET:_itl / SET:_itv are accepted and the emulator
cycles motor/rem field numbers and frame counters accordingly, but all
fields sample the same angle grid (no sub-bin angular offsets). Driver
interlace bookkeeping is exercised; angular super-resolution is not.

Loopback E2E (no extra NIC needed):
  ros2 run hils_bridge_lidar_hokuyo_yvt35lx yvt35lx_emulator_node \
      --ros-args -p device_ip:=127.0.0.1 -p host_ip:=127.0.0.1 \
      -p pointcloud_topic:=/sim_points

CAUTION: urg3d_library resynchronizes by scanning for the "VSSP" magic,
so corruption faults are survivable, but a corrupted length field makes
the parser swallow up to 64 KiB of following stream — expect gaps, not
just single-packet loss, when injecting corruption on TCP.
"""

import socket
import struct
import threading
import time

import numpy as np

import rclpy
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from hils_bridge_base.udp_emulator_base import UdpEmulatorBase
from hils_bridge_lidar_hokuyo_yvt35lx import vssp_protocol as vp


class _ClientSession:
    """One accepted VSSP TCP connection: command reader + line streamer."""

    def __init__(self, node, conn: socket.socket, addr):
        self._node = node
        self._conn = conn
        self.addr = addr
        self.alive = True
        self._send_lock = threading.Lock()
        self._rx = b''
        # Measurement stream state (DAT:xx=0/1). Per-connection: a
        # closed socket ends its streams, as with the real sensor.
        self.ro = False
        self.ri = False
        self.ax = False
        self.rev = 0  # revolutions sent (drives field/frame counters)
        self._reader = threading.Thread(
            target=self._reader_loop, daemon=True)
        self._streamer = threading.Thread(
            target=self._stream_loop, daemon=True)

    def start(self):
        self._reader.start()
        self._streamer.start()

    def close(self):
        self.alive = False
        try:
            self._conn.close()
        except OSError:
            pass

    def send_bytes(self, data: bytes) -> bool:
        if not self.alive:
            return False
        try:
            with self._send_lock:
                self._conn.sendall(data)
            return True
        except OSError:
            self.alive = False
            return False

    # ── Command reader ──

    def _reader_loop(self):
        self._conn.settimeout(0.5)
        while self.alive and not self._node.quitting:
            try:
                data = self._conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            self._rx += data
            while b'\n' in self._rx:
                line, self._rx = self._rx.split(b'\n', 1)
                command = line.decode('ascii', errors='replace').strip('\r')
                if command:
                    self._node.handle_command(self, command)
        self.alive = False

    # ── Measurement streamer ──

    def _stream_loop(self):
        while self.alive and not self._node.quitting:
            period = 1.0 / self._node.scan_rate_hz
            tick_start = time.monotonic()
            if self.ro or self.ri or self.ax:
                self._send_revolution(period)
            # Keep the revolution cadence even when send took time
            elapsed = time.monotonic() - tick_start
            time.sleep(max(0.005, period - elapsed))

    def _send_revolution(self, period: float):
        node = self._node
        motor_intl, rem_intl = node.motor_interlace, node.rem_interlace
        motor_field = self.rev % motor_intl
        rem_field = (self.rev // motor_intl) % rem_intl
        frame = (self.rev // (motor_intl * rem_intl)) % 256

        base_ms = node.sensor_ms()
        period_ms = period * 1000.0

        if self.ro or self.ri:
            # Fresh noise for this revolution: a real sensor measures
            # each scan independently, so two frames of a static scene
            # must not carry identical ranges.
            range_grid, intens_grid = node.noise.apply(*node.grids)
            geom = node.geometry
            line_ms = period_ms / vp.LINE_COUNT
            for li in range(vp.LINE_COUNT):
                ts_head = int(base_ms + li * line_ms)
                ts_tail = int(base_ms + (li + 1) * line_ms)
                for with_intensity in ((True,) if self.ri else ()) + \
                                      ((False,) if self.ro else ()):
                    pkt = vp.build_range_packet(
                        with_intensity=with_intensity,
                        frame=frame, motor_field=motor_field,
                        rem_field=rem_field, rem_interlace=rem_intl,
                        line=li, ts_head_ms=ts_head, ts_tail_ms=ts_tail,
                        head_ratio=geom.head_ratios[li],
                        tail_ratio=geom.tail_ratios[li],
                        ranges_mm=range_grid[li],
                        intensities=intens_grid[li],
                        ts_ms=ts_tail)
                    if not node.send_tcp(self, pkt, channel='data'):
                        return
            node.count_revolution()

        if self.ax:
            n_records = max(1, min(10, int(period_ms // 10)))
            data_ms = max(1, int(period_ms / n_records))
            records = [node.aux_record] * n_records
            pkt = vp.build_ax_packet(int(base_ms), records, data_ms)
            node.send_tcp(self, pkt, channel='imu')

        self.rev += 1


class Yvt35lxEmulatorNode(UdpEmulatorBase):
    """ROS2 node that serves PointCloud2 data as a VSSP 2.1 TCP sensor."""

    def __init__(self):
        super().__init__(
            node_name='hils_yvt35lx_emulator',
            default_device_ip='192.168.0.10',
            default_host_ip='192.168.0.15')

        # ── Parameters ──
        self.declare_parameter('tcp_port', vp.VSSP_PORT,
            ParameterDescriptor(description='VSSP TCP port (sensor: 10940)'))
        self.declare_parameter('pointcloud_topic', '/sim_points',
            ParameterDescriptor(description='PointCloud2 topic from simulation'))
        self.declare_parameter('scan_rate_hz', 10.0,
            ParameterDescriptor(
                description='Motor revolutions per second (frame rate at '
                            'interlace 1)'))
        self.declare_parameter('horizontal_fov_deg', 210.0,
            ParameterDescriptor(description='Horizontal field of view'))
        self.declare_parameter('vertical_fov_min_deg', -20.0,
            ParameterDescriptor(description='Lower vertical scan angle'))
        self.declare_parameter('vertical_fov_max_deg', 20.0,
            ParameterDescriptor(description='Upper vertical scan angle'))
        # Sensor noise. Off by default: the simulation feeding this
        # emulator normally models its own sensor noise, and applying it
        # on both sides would misrepresent what the driver really sees.
        self.declare_parameter('range_noise_sigma_m', 0.0,
            ParameterDescriptor(
                description='Std-dev of range noise along the beam, in '
                            'metres. 0 = no noise.'))
        self.declare_parameter('dropout_probability', 0.0,
            ParameterDescriptor(
                description='Probability that a spot reports no echo. '
                            '0 = every spot returns.'))
        self.declare_parameter('noise_seed', 0,
            ParameterDescriptor(
                description='Seed for the noise generator, so a run can '
                            'be reproduced.'))
        self.declare_parameter('serial_number', 'H0000001',
            ParameterDescriptor(description='Emulated serial number'))
        self.declare_parameter('firmware_version', '1.2.0-hils',
            ParameterDescriptor(description='Reported firmware version'))

        self.geometry = vp.ScanGeometry(
            h_fov_deg=self.get_parameter('horizontal_fov_deg').value,
            v_min_deg=self.get_parameter('vertical_fov_min_deg').value,
            v_max_deg=self.get_parameter('vertical_fov_max_deg').value)
        self.noise = vp.RangeNoise(
            sigma_m=self.get_parameter('range_noise_sigma_m').value,
            dropout_probability=self.get_parameter(
                'dropout_probability').value,
            seed=self.get_parameter('noise_seed').value)

        # ── Sensor state ──
        self.quitting = False
        self.motor_interlace = 1
        self.rem_interlace = 1
        self.grids = (
            np.zeros((vp.LINE_COUNT, vp.SPOT_COUNT), dtype=np.uint16),
            np.zeros((vp.LINE_COUNT, vp.SPOT_COUNT), dtype=np.uint16))
        self.aux_record = vp.aux_record_raw()
        self._epoch = time.monotonic()
        self._motor_table = vp.motor_ratio_table()
        self._vertical_table = vp.vertical_angle_table(
            self.get_parameter('vertical_fov_min_deg').value,
            self.get_parameter('vertical_fov_max_deg').value)

        self._sessions = []
        self._sessions_lock = threading.Lock()
        self._cloud_count = 0
        self._rev_count = 0

        # ── TCP server ──
        port = self.get_parameter('tcp_port').value
        self._listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listen_sock.bind((self.device_ip, port))
        self._listen_sock.listen(4)
        self._listen_sock.settimeout(0.5)
        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True)
        self._accept_thread.start()

        # ── Subscription ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.create_subscription(
            PointCloud2, self.get_parameter('pointcloud_topic').value,
            self._pointcloud_callback, sensor_qos)

        self.get_logger().info(
            f'YVT-35LX emulator: VSSP TCP {self.device_ip}:{port}, '
            f'{vp.LINE_COUNT} lines x {vp.SPOT_COUNT} spots, '
            f'scan {self.get_parameter("scan_rate_hz").value} Hz, '
            f'source {self.get_parameter("pointcloud_topic").value}')
        if self.noise.enabled:
            self.get_logger().info(
                f'sensor noise ON: range sigma '
                f'{self.get_parameter("range_noise_sigma_m").value} m, '
                f'dropout {self.get_parameter("dropout_probability").value}, '
                f'seed {self.get_parameter("noise_seed").value} - disable it '
                f'if the simulation already models sensor noise')

    # ── Properties used by sessions ──

    @property
    def scan_rate_hz(self) -> float:
        rate = self.get_parameter('scan_rate_hz').value
        return rate if rate > 0.1 else 10.0

    def sensor_ms(self) -> float:
        """Sensor-clock milliseconds (u32 wrap handled at pack time)."""
        return (time.monotonic() - self._epoch) * 1000.0

    def count_revolution(self):
        self._rev_count += 1
        self.mark_sent()

    # ── TCP server ──

    def _accept_loop(self):
        while not self.quitting:
            try:
                conn, addr = self._listen_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            session = _ClientSession(self, conn, addr)
            with self._sessions_lock:
                self._sessions = [c for c in self._sessions if c.alive]
                self._sessions.append(session)
            session.start()
            self.get_logger().info(f'VSSP client connected: {addr}')

    def send_tcp(self, session: _ClientSession, data: bytes,
                 *, channel: str) -> bool:
        """Send one VSSP packet through the state gate + fault pipeline.

        TCP preserves ordering, so a delay fault sleeps in the calling
        stream/reader thread — everything behind the delayed packet is
        delayed too, exactly as on a congested link.
        """
        if not self.device_state.allows(channel):
            return True
        for pkt in self.fault_pipeline.apply(bytes(data), channel):
            if pkt.delay_s > 0.0:
                time.sleep(min(pkt.delay_s, 5.0))
            if not session.send_bytes(pkt.data):
                return False
        return True

    # ── Command handling ──

    def handle_command(self, session: _ClientSession, command: str):
        ts = self.sensor_ms()
        pkt = None

        if command == 'VER':
            pkt = vp.build_ver_response(
                'HOKUYO AUTOMATIC CO.,LTD.', 'YVT-35LX',
                self.get_parameter('serial_number').value,
                self.get_parameter('firmware_version').value,
                'VSSP2.1', ts)
        elif command == 'GET:stat':
            pkt = vp.build_response(command, vp.STATUS_OK,
                                    '_ro=000\n_ri=000\n_ax=000\n', ts)
        elif command == 'GET:_itl':
            pkt = vp.build_response(command, vp.STATUS_OK,
                                    f'0,{self.motor_interlace:02d}\n', ts)
        elif command == 'GET:_itv':
            pkt = vp.build_response(command, vp.STATUS_OK,
                                    f'0,{self.rem_interlace:02d}\n', ts)
        elif command == 'GET:tblh':
            pkt = vp.encode_angle_table(command, self._motor_table, ts)
        elif command.startswith('GET:tv') and len(command) == 8 \
                and command[6:8].isdigit():
            # All rem fields share the same base table (see module doc).
            pkt = vp.encode_angle_table(command, self._vertical_table, ts)
        elif command.startswith('SET:_itl=0,'):
            pkt = self._set_interlace(command, 'motor',
                                      vp.MAX_MOTOR_INTERLACE, ts)
        elif command.startswith('SET:_itv=0,'):
            pkt = self._set_interlace(command, 'rem',
                                      vp.MAX_REM_INTERLACE, ts)
        elif command.startswith('DAT:') and len(command) == 8 \
                and command[4:6] in ('ro', 'ri', 'ax') \
                and command[6] == '=' and command[7] in '01':
            enable = command[7] == '1'
            setattr(session, command[4:6], enable)
            if enable:
                session.rev = 0  # start clean at frame/field/line zero
            pkt = vp.build_response(command, vp.STATUS_OK, '', ts)
            self.get_logger().info(
                f'{session.addr}: {command[4:6]} '
                f'{"start" if enable else "stop"}')
        elif command == 'RST':
            self.motor_interlace = 1
            self.rem_interlace = 1
            session.ro = session.ri = session.ax = False
            pkt = vp.build_response(command, vp.STATUS_OK, '', ts)
        else:
            self.get_logger().warning(f'Unknown VSSP command: {command!r}')
            pkt = vp.build_response(command, vp.STATUS_ERROR, '', ts)

        if pkt is not None:
            self.send_tcp(session, pkt, channel='command')

    def _set_interlace(self, command: str, which: str, max_count: int,
                       ts: float):
        try:
            count = int(command.split(',', 1)[1])
        except ValueError:
            return vp.build_response(command, vp.STATUS_ERROR, '', ts)
        if not 1 <= count <= max_count:
            return vp.build_response(command, vp.STATUS_ERROR, '', ts)
        if which == 'motor':
            self.motor_interlace = count
        else:
            self.rem_interlace = count
        return vp.build_response(command, vp.STATUS_OK, '', ts)

    # ── Point cloud input ──

    def _pointcloud_callback(self, msg: PointCloud2):
        try:
            has_intensity = any(f.name == 'intensity' for f in msg.fields)
            fields = ['x', 'y', 'z'] + (['intensity'] if has_intensity else [])
            pts = point_cloud2.read_points(
                msg, field_names=fields, skip_nans=True)
        except Exception as e:
            self.get_logger().warn(f'Point extraction failed: {e}')
            return
        if pts is None or len(pts) == 0:
            return
        x = pts['x'].astype(np.float64)
        y = pts['y'].astype(np.float64)
        z = pts['z'].astype(np.float64)
        inten = (pts['intensity'].astype(np.float64) if has_intensity
                 else np.zeros(len(x)))
        # Tuple assignment is atomic; streamer threads always see a
        # consistent (range, intensity) pair. Noise is not applied here:
        # it is drawn per revolution, so each scan is an independent
        # measurement even when the source publishes more slowly.
        self.grids = self.geometry.bin_pointcloud(x, y, z, inten)
        self._cloud_count += 1

    # ── Stats ──

    def _stats_callback(self):
        with self._sessions_lock:
            n_clients = sum(1 for c in self._sessions if c.alive)
        self.get_logger().info(
            f'Status: clients={n_clients}, clouds={self._cloud_count}, '
            f'revolutions={self._rev_count}, '
            f'interlace=({self.motor_interlace},{self.rem_interlace})')

    # ── Cleanup ──

    def destroy_node(self):
        self.quitting = True
        try:
            self._listen_sock.close()
        except OSError:
            pass
        with self._sessions_lock:
            for client in self._sessions:
                client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Yvt35lxEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
