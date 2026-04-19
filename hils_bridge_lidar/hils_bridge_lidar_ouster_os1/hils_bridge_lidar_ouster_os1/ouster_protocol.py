"""
Ouster OS1 protocol helpers.

Implements the LEGACY UDP packet format and the HTTP REST API
used by modern ouster_ros driver (based on Ouster SDK with CurlClient).

References:
  - Ouster OS1 User Manual (LEGACY UDP format)
  - ouster-lidar/ouster-sdk CurlClient

HTTP REST API (port 80 on the sensor):
  GET  /api/v1/sensor/metadata              -> full metadata blob
  GET  /api/v1/sensor/metadata/sensor_info  -> sensor_info sub-section
  GET  /api/v1/sensor/metadata/beam_intrinsics
  GET  /api/v1/sensor/metadata/imu_intrinsics
  GET  /api/v1/sensor/metadata/lidar_intrinsics
  GET  /api/v1/sensor/metadata/lidar_data_format
  GET  /api/v1/sensor/metadata/calibration_status
  GET  /api/v1/sensor/config                -> active config params
  PUT  /api/v1/sensor/config                -> update config params (JSON body)
  POST /api/v1/sensor/cmd/set_udp_dest_auto -> auto-configure UDP dest
  POST /api/v1/sensor/cmd/reinitialize      -> reinit sensor

LEGACY Lidar UDP packet:
  16 column blocks back-to-back, no packet-level header.
  Each column block = column_header(16B) + channel_data(N*12B) + trailer(4B)
    column_header: timestamp_ns (u64) + measurement_id (u16) + frame_id (u16)
                   + encoder_count (u32)
    channel_data per pixel (12B): range_mm (u32) + reflectivity (u16)
                                  + signal (u16) + near_ir (u16) + unused (u16)
    trailer (4B): packet status uint32. 0xFFFFFFFF = valid, 0 = invalid.

IMU packet (48B): diag_ts (u64) + accel_ts (u64) + gyro_ts (u64)
                  + accel_xyz (3*f32) + gyro_xyz (3*f32)
"""

import json
import socket
import struct
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
from typing import List
from urllib.parse import urlparse, parse_qs

# ── Protocol constants ──

OUSTER_LIDAR_PORT = 7502
OUSTER_IMU_PORT = 7503
OUSTER_HTTP_PORT = 80

COLUMNS_PER_PACKET = 16         # LEGACY: 16 column blocks per UDP packet
COLUMN_HEADER_SIZE = 16
CHANNEL_DATA_SIZE = 12
COLUMN_TRAILER_SIZE = 4
COLUMN_STATUS_VALID = 0xFFFFFFFF

IMU_PACKET_SIZE = 48

# Encoder ticks per full rotation (Ouster convention)
ENCODER_TICKS_PER_REV = 90112

# ── Beam angle tables (degrees) ──

BEAM_ANGLES_16 = [
    16.611, 11.032, 5.579, 0.168, -5.125, -10.502, -15.880, -21.207,
    14.822, 9.206, 3.721, -1.673, -7.147, -12.526, -17.887, -23.339,
]


def default_beam_angles(n_channels: int) -> List[float]:
    """Return representative beam altitude angles (deg) for the given count."""
    if n_channels == 16:
        return list(BEAM_ANGLES_16)
    import numpy as np
    if n_channels == 32:
        return np.linspace(22.5, -22.5, 32).tolist()
    if n_channels == 64:
        return np.linspace(16.6, -16.6, 64).tolist()
    return np.linspace(22.5, -22.5, n_channels).tolist()


# ── Packet building ──

def build_column_block(timestamp_ns: int, measurement_id: int, frame_id: int,
                       encoder_count: int, channel_data: bytes,
                       status: int = COLUMN_STATUS_VALID) -> bytes:
    """Build a single column block (header + channel data + trailer)."""
    header = struct.pack('<QHHI',
                         timestamp_ns & 0xFFFFFFFFFFFFFFFF,
                         measurement_id & 0xFFFF,
                         frame_id & 0xFFFF,
                         encoder_count & 0xFFFFFFFF)
    trailer = struct.pack('<I', status & 0xFFFFFFFF)
    return header + channel_data + trailer


def build_channel_data(range_mm: int, reflectivity: int = 0,
                       signal: int = 0, near_ir: int = 0) -> bytes:
    """Build one 12-byte channel data entry."""
    return struct.pack('<IHHHH',
                       range_mm & 0xFFFFFFFF,
                       reflectivity & 0xFFFF,
                       signal & 0xFFFF,
                       near_ir & 0xFFFF,
                       0)  # unused


def build_imu_packet(timestamp_ns: int,
                     accel_x: float, accel_y: float, accel_z: float,
                     gyro_x: float, gyro_y: float, gyro_z: float) -> bytes:
    """Build a 48-byte Ouster IMU UDP packet."""
    return struct.pack('<QQQffffff',
                       timestamp_ns & 0xFFFFFFFFFFFFFFFF,  # diag_ts
                       timestamp_ns & 0xFFFFFFFFFFFFFFFF,  # accel_ts
                       timestamp_ns & 0xFFFFFFFFFFFFFFFF,  # gyro_ts
                       accel_x, accel_y, accel_z,
                       gyro_x, gyro_y, gyro_z)


# ── Metadata dictionaries ──

def build_sensor_info(n_channels: int, lidar_mode: str,
                      serial: str, prod_pn: str, fw_rev: str,
                      udp_dest: str) -> dict:
    """Top-level 'sensor_info' block."""
    prod_line = f"OS-1-{n_channels}"
    return {
        "prod_line": prod_line,
        "prod_pn": prod_pn,
        "prod_sn": serial,
        "fw_rev": fw_rev,
        "status": "RUNNING",
        "mode": lidar_mode,
        "initialization_id": 390,
        "build_rev": fw_rev,
        "image_rev": fw_rev,
        "proto_rev": "v1.3.1",
        "build_date": "2024-01-01T00:00:00Z",
        "base_pn": prod_pn,
        "base_sn": serial,
        "status_flags": 0,
    }


def build_beam_intrinsics(beam_angles: List[float]) -> dict:
    n_channels = len(beam_angles)
    return {
        "beam_altitude_angles": list(beam_angles),
        "beam_azimuth_angles": [0.0] * n_channels,
        "lidar_origin_to_beam_origin_mm": 27.67,
        "beam_to_lidar_transform": [
            1.0, 0.0, 0.0, 27.67,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ],
    }


def build_imu_intrinsics() -> dict:
    return {
        "imu_to_sensor_transform": [
            1.0, 0.0, 0.0, 6.253,
            0.0, 1.0, 0.0, -11.775,
            0.0, 0.0, 1.0, 7.645,
            0.0, 0.0, 0.0, 1.0,
        ],
    }


def build_lidar_intrinsics() -> dict:
    return {
        "lidar_to_sensor_transform": [
            -1.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 36.18,
            0.0, 0.0, 0.0, 1.0,
        ],
    }


def build_calibration_status() -> dict:
    return {
        "reflectivity": {"valid": True, "timestamp": ""},
    }


def build_lidar_data_format(n_channels: int, columns_per_frame: int) -> dict:
    return {
        "pixels_per_column": n_channels,
        "columns_per_packet": COLUMNS_PER_PACKET,
        "columns_per_frame": columns_per_frame,
        "pixel_shift_by_row": [0] * n_channels,
        "column_window": [0, columns_per_frame - 1],
        "udp_profile_lidar": "LEGACY",
        "udp_profile_imu": "LEGACY",
        "fps": 10,
    }


def build_config_params(udp_dest: str, lidar_mode: str) -> dict:
    return {
        "udp_dest": udp_dest,
        "udp_port_lidar": OUSTER_LIDAR_PORT,
        "udp_port_imu": OUSTER_IMU_PORT,
        "lidar_mode": lidar_mode,
        "timestamp_mode": "TIME_FROM_INTERNAL_OSC",
        "sync_pulse_in_polarity": "ACTIVE_HIGH",
        "nmea_in_polarity": "ACTIVE_HIGH",
        "nmea_ignore_valid_char": 0,
        "nmea_baud_rate": "BAUD_9600",
        "nmea_leap_seconds": 0,
        "multipurpose_io_mode": "OFF",
        "sync_pulse_out_polarity": "ACTIVE_HIGH",
        "sync_pulse_out_frequency": 1,
        "sync_pulse_out_angle": 360,
        "sync_pulse_out_pulse_width": 10,
        "auto_start_flag": 1,
        "operating_mode": "NORMAL",
        "phase_lock_enable": False,
        "phase_lock_offset": 0,
        "columns_per_packet": COLUMNS_PER_PACKET,
        "udp_profile_lidar": "LEGACY",
        "udp_profile_imu": "LEGACY",
    }


def build_full_metadata(n_channels: int, lidar_mode: str, columns_per_frame: int,
                        serial: str, prod_pn: str, fw_rev: str,
                        udp_dest: str, beam_angles: List[float]) -> dict:
    """Return the full metadata blob returned by /api/v1/sensor/metadata."""
    return {
        "sensor_info": build_sensor_info(
            n_channels, lidar_mode, serial, prod_pn, fw_rev, udp_dest),
        "beam_intrinsics": build_beam_intrinsics(beam_angles),
        "imu_intrinsics": build_imu_intrinsics(),
        "lidar_intrinsics": build_lidar_intrinsics(),
        "calibration_status": build_calibration_status(),
        "config_params": build_config_params(udp_dest, lidar_mode),
        "lidar_data_format": build_lidar_data_format(
            n_channels, columns_per_frame),
    }


# ── HTTP REST API server ──

class _ThreadingHTTPServer(ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class _OusterApiHandler(BaseHTTPRequestHandler):
    """Handler for Ouster's HTTP REST API.

    The server attaches a `metadata_store` and `on_config_change` callback
    via attributes on the server instance.
    """

    # Silence default HTTP logging to stderr; use the ROS logger instead.
    def log_message(self, format, *args):
        if hasattr(self.server, 'logger') and self.server.logger is not None:
            try:
                self.server.logger.debug(
                    f'{self.address_string()} - {format % args}')
            except Exception:
                pass

    def _send_json(self, obj, status=200):
        body = json.dumps(obj).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, text, status=200):
        body = text.encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'text/plain')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _path(self):
        return urlparse(self.path).path.rstrip('/')

    def _query(self):
        return parse_qs(urlparse(self.path).query)

    def _read_body(self):
        length = int(self.headers.get('Content-Length', 0))
        return self.rfile.read(length) if length > 0 else b''

    def _handle_metadata_get(self, path):
        meta = self.server.metadata_store()
        if path == '/api/v1/sensor/metadata':
            self._send_json(meta); return True
        if path == '/api/v1/sensor/metadata/sensor_info':
            self._send_json(meta['sensor_info']); return True
        if path == '/api/v1/sensor/metadata/beam_intrinsics':
            self._send_json(meta['beam_intrinsics']); return True
        if path == '/api/v1/sensor/metadata/imu_intrinsics':
            self._send_json(meta['imu_intrinsics']); return True
        if path == '/api/v1/sensor/metadata/lidar_intrinsics':
            self._send_json(meta['lidar_intrinsics']); return True
        if path == '/api/v1/sensor/metadata/lidar_data_format':
            self._send_json(meta['lidar_data_format']); return True
        if path == '/api/v1/sensor/metadata/calibration_status':
            self._send_json(meta['calibration_status']); return True
        if path == '/api/v1/sensor/config':
            self._send_json(meta['config_params']); return True
        if path == '/api/v1/sensor/time':
            self._send_json({
                'timestamp': {
                    'mode': 'TIME_FROM_INTERNAL_OSC',
                    'time': 0,
                }
            })
            return True
        if path == '/api/v1/system/firmware':
            self._send_json({'fw': meta['sensor_info'].get('fw_rev', 'v2.5.2')})
            return True
        if path == '/api/v1/user/data':
            # User-defined data storage on the sensor. HILS returns empty.
            self._send_json('')
            return True
        return False

    def _handle_config_update(self, raw):
        try:
            updates = json.loads(raw) if raw else {}
        except Exception as e:
            self.send_error(400, f'Invalid JSON: {e}')
            return
        if self.server.on_config_change is not None:
            try:
                self.server.on_config_change(updates)
            except Exception:
                pass
        # Ouster SDK expects empty object from config update endpoints
        self._send_json({})

    def _log(self, msg, level='info'):
        logger = getattr(self.server, 'logger', None)
        if logger is None:
            return
        try:
            getattr(logger, level)(msg)
        except Exception:
            pass

    def _handle_command(self, path, query=None):
        """Commands are idempotent for HILS; accept via GET/PUT/POST.

        The Ouster SDK validates cmd responses against exact expected bodies.
        Response conventions observed from real sensor behavior:
          - action commands (e.g. set_udp_dest_auto, reinitialize) return `{}`
          - setter commands (set_config_param, save_config_params) return the
            command name as a JSON string (e.g. "set_config_param")
          - getter commands return the requested data
        """
        meta = self.server.metadata_store()
        query = query or {}

        # Surface every command invocation at INFO so the user can trace
        # exactly what the driver is requesting.
        self._log(f'[HTTP cmd] {self.command} {path} query={query}')

        if path == '/api/v1/sensor/cmd/set_udp_dest_auto':
            self._send_json({}); return True
        if path == '/api/v1/sensor/cmd/reinitialize':
            self._send_json({}); return True

        # get_config_param ?args=active|staged -> return config_params
        # In HILS we return the same dict for both active and staged.
        if path == '/api/v1/sensor/cmd/get_config_param':
            self._send_json(meta['config_params']); return True

        # set_config_param ?args=.+<JSON>  -> apply updates, echo cmd name
        if path == '/api/v1/sensor/cmd/set_config_param':
            self._apply_config_args(query)
            self._send_json('set_config_param'); return True

        # save_config_params: persist active config (no-op in HILS)
        if path == '/api/v1/sensor/cmd/save_config_params':
            self._send_json('save_config_params'); return True

        return False

    def _apply_config_args(self, query):
        """Extract JSON updates from the 'args' query parameter and apply.

        Ouster's set_config_param sends updates as:
          ?args=.+<url-encoded-json>
        where the leading '.+' is a prefix the SDK adds.

        GOTCHA: parse_qs() decodes '+' as space (x-www-form-urlencoded
        convention), so the received arg often starts with '. ' (dot+space)
        instead of '.+'. The Ouster SDK does not percent-encode its '+',
        so we must accept both forms.
        """
        args_list = query.get('args', [])
        if not args_list:
            self._log('[set_config_param] no args in query')
            return
        arg = args_list[0]
        self._log(f'[set_config_param] raw args ({len(arg)} chars): {arg[:200]}'
                  f'{"..." if len(arg) > 200 else ""}')
        # Strip leading '.+' prefix (or '. ' after parse_qs URL-decoding)
        if arg.startswith('.+') or arg.startswith('. '):
            arg = arg[2:]
        # Find the first '{' and drop anything before it as a last resort
        brace = arg.find('{')
        if brace > 0:
            arg = arg[brace:]
        arg = arg.strip()
        if not arg:
            self._log('[set_config_param] args empty after prefix strip')
            return
        try:
            updates = json.loads(arg)
        except Exception as e:
            self._log(f'[set_config_param] JSON parse failed: {e}', 'warn')
            return
        if not isinstance(updates, dict):
            self._log(f'[set_config_param] updates not a dict: '
                      f'{type(updates).__name__}', 'warn')
            return
        self._log(f'[set_config_param] parsed {len(updates)} keys: '
                  f'{sorted(updates.keys())}')
        if self.server.on_config_change is not None:
            try:
                self.server.on_config_change(updates)
            except Exception as e:
                self._log(f'[set_config_param] on_config_change error: {e}',
                          'warn')

    def do_GET(self):
        path = self._path()
        if self._handle_metadata_get(path):
            self._log(f'[HTTP get] {path}')
            return
        if self._handle_command(path, self._query()):
            return
        self._log(f'[HTTP 404] GET {self.path}', 'warn')
        self.send_error(404, f'Not Found: {self.path}')

    def do_PUT(self):
        path = self._path()
        raw = self._read_body()
        if path == '/api/v1/sensor/config':
            self._handle_config_update(raw)
            return
        if path == '/api/v1/user/data':
            # Accept user-defined data write; no-op in HILS
            self._send_json('')
            return
        if self._handle_command(path, self._query()):
            return
        self.send_error(404, f'Not Found: {self.path}')

    def do_POST(self):
        path = self._path()
        raw = self._read_body()
        if path == '/api/v1/sensor/config':
            self._handle_config_update(raw)
            return
        if path == '/api/v1/user/data':
            self._send_json('')
            return
        if self._handle_command(path, self._query()):
            return
        self.send_error(404, f'Not Found: {self.path}')

    def do_DELETE(self):
        path = self._path()
        if path == '/api/v1/user/data':
            self._send_json('')
            return
        self.send_error(404, f'Not Found: {self.path}')


class HttpApiServer:
    """Minimal Ouster HTTP REST API server running in a background thread.

    Must bind on the emulated sensor IP, port 80. To bind to port 80 as an
    unprivileged user inside a Docker container, set the sysctl
    `net.ipv4.ip_unprivileged_port_start=80`.
    """

    def __init__(self, bind_ip: str,
                 metadata_provider,
                 on_config_change=None,
                 logger=None):
        """
        Args:
            bind_ip: IP to bind the HTTP server to (the emulated sensor IP).
            metadata_provider: callable returning the current full metadata dict.
            on_config_change: callable(updates_dict) invoked on PUT config.
            logger: optional ROS logger.
        """
        self._server = _ThreadingHTTPServer(
            (bind_ip, OUSTER_HTTP_PORT), _OusterApiHandler)
        # Attach provider/callbacks to the server instance so handlers can access
        self._server.metadata_store = metadata_provider
        self._server.on_config_change = on_config_change
        self._server.logger = logger
        self._thread = None

    def start(self):
        self._thread = threading.Thread(
            target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        try:
            self._server.shutdown()
            self._server.server_close()
        except Exception:
            pass
