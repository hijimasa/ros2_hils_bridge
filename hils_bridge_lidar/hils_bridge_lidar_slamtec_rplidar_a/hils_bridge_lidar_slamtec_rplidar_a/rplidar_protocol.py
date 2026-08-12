"""SLAMTEC RPLIDAR serial protocol encode/decode (device/slave side).

Python port of REACT-simulator's rplidar_emulator
(rplidar_protocol.{hpp,cpp} + the request parsing in
rplidar_emulator_node.cpp). This is the sensor side of the protocol
that the SLAMTEC SDK / rplidar_ros driver (host) speaks:

    Request  (host -> sensor):
        [0xA5][cmd]                                  no-payload command
        [0xA5][cmd|0x80][size][payload...][checksum] payload command
        checksum = XOR of every preceding byte (frame XORs to 0).

    Response descriptor (sensor -> host, once per request):
        [0xA5][0x5A][size_q30_subtype:4 LE][type:1]
        size_q30_subtype: bits 0-29 = data length per response packet,
        bits 30-31 = send mode (0 = single response, 1 = continuous).

    Scan stream (sensor -> host, after a SCAN descriptor, until STOP):
        5-byte standard measurement nodes:
        [sync_quality][angle_q6_checkbit:2 LE][distance_q2:2 LE]
        sync_quality: bit0 = syncbit (1 on the first node of each
        revolution), bit1 = inverted syncbit, bits 2-7 = quality.
        angle_q6_checkbit: bit0 = checkbit (always 1), bits 1-15 =
        angle in degrees * 64. distance_q2 = distance in mm * 4
        (0 = invalid measurement).

Like the C++ emulator, request checksums are NOT verified on parse
(resync is by scanning for the 0xA5 sync byte); xor_checksum() /
build_command() are provided so tests and master-side smoke scripts
can emit well-formed requests.

No ROS imports: everything here is unit-testable with plain pytest.
"""

import math
import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Protocol constants
CMD_SYNC_BYTE = 0xA5
ANS_SYNC_BYTE1 = 0xA5
ANS_SYNC_BYTE2 = 0x5A
CMDFLAG_HAS_PAYLOAD = 0x80

# Command codes
CMD_STOP = 0x25
CMD_SCAN = 0x20
CMD_FORCE_SCAN = 0x21
CMD_RESET = 0x40
CMD_GET_DEVICE_INFO = 0x50
CMD_GET_DEVICE_HEALTH = 0x52
CMD_GET_SAMPLERATE = 0x59
CMD_EXPRESS_SCAN = 0x82
CMD_HQ_SCAN = 0x83
CMD_GET_LIDAR_CONF = 0x84
CMD_SET_MOTOR_PWM = 0xF0
CMD_HQ_MOTOR_SPEED = 0xA8

# Response types
ANS_TYPE_DEVINFO = 0x04
ANS_TYPE_DEVHEALTH = 0x06
ANS_TYPE_SAMPLERATE = 0x15
ANS_TYPE_MEASUREMENT = 0x81
ANS_TYPE_MEASUREMENT_CAPSULED = 0x82
ANS_TYPE_MEASUREMENT_HQ = 0x83
ANS_TYPE_GET_LIDAR_CONF = 0x20

# Send modes (bits 30-31 of size_q30_subtype)
SEND_MODE_SINGLE = 0
SEND_MODE_CONTINUOUS = 1

# LIDAR configuration types (GET_LIDAR_CONF payload)
LIDAR_CONF_DESIRED_ROT_FREQ = 0x00000001
LIDAR_CONF_SCAN_MODE_COUNT = 0x00000070
LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE = 0x00000071
LIDAR_CONF_SCAN_MODE_MAX_DISTANCE = 0x00000074
LIDAR_CONF_SCAN_MODE_ANS_TYPE = 0x00000075
LIDAR_CONF_LIDAR_MAC_ADDR = 0x00000079
LIDAR_CONF_SCAN_MODE_TYPICAL = 0x0000007C
LIDAR_CONF_SCAN_MODE_NAME = 0x0000007F

# Health status
STATUS_OK = 0x00
STATUS_WARNING = 0x01
STATUS_ERROR = 0x02

# Data length per scan-stream packet, per answer type (from the SDK's
# response struct sizes; the C++ emulator streams standard 5-byte
# nodes regardless of the advertised type).
_SCAN_DATA_SIZE = {
    ANS_TYPE_MEASUREMENT: 5,
    ANS_TYPE_MEASUREMENT_CAPSULED: 84,
    ANS_TYPE_MEASUREMENT_HQ: 781,
}

MEASUREMENT_NODE_BYTES = 5

# Values the C++ emulator reports for its single "Standard" scan mode.
DEFAULT_DESIRED_PWM_REF = 600
DEFAULT_DESIRED_RPM = 600
DEFAULT_US_PER_SAMPLE = 130          # microseconds per sample
DEFAULT_MAX_DISTANCE_M = 30          # metres
DEFAULT_MAC_ADDR = bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55])
DEFAULT_SCAN_MODE_NAME = 'Standard'

# distance_q2 is uint16: clamp instead of overflowing (~16.38 m max).
# Zero means "invalid" to the SDK, and if ALL points are zero
# ascendScanData returns SL_RESULT_OPERATION_FAIL.
_DISTANCE_Q2_MAX = 65534


@dataclass
class DeviceInfo:
    """GET_DEVICE_INFO reply payload (C++ emulator defaults: S2)."""

    model: int = 0x61
    firmware_version: int = 0x0118  # v1.24 (major.minor packed in 16 bit)
    hardware_version: int = 0x07
    serial_number: bytes = b'EMULATOR00000001'


@dataclass
class ScanPoint:
    """Single scan point (ROS units: rad / m)."""

    angle: float = 0.0      # radians (ROS CCW convention)
    distance: float = 0.0   # metres
    quality: int = 0
    valid: bool = False


# ---------------------------------------------------------------------------
# Requests (host -> sensor)
# ---------------------------------------------------------------------------

def xor_checksum(data: bytes) -> int:
    """XOR of all bytes (SLAMTEC request checksum: frame XORs to 0)."""
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_command(cmd: int, payload: bytes = b'') -> bytes:
    """Build a master-side request frame (for tests / smoke scripts).

    Commands with bit 7 set carry [size][payload][checksum]; the
    command constants already include the flag (e.g. GET_LIDAR_CONF =
    0x84), matching the SDK.
    """
    if cmd & CMDFLAG_HAS_PAYLOAD:
        frame = bytearray([CMD_SYNC_BYTE, cmd, len(payload)])
        frame.extend(payload)
        frame.append(xor_checksum(bytes(frame)))
        return bytes(frame)
    if payload:
        raise ValueError(f'command 0x{cmd:02X} takes no payload')
    return bytes([CMD_SYNC_BYTE, cmd])


def extract_command(rx_buffer: bytearray) -> Optional[Tuple[int, bytes]]:
    """Pop one request off the front of rx_buffer.

    Returns (cmd, payload) or None when more bytes are needed.
    Resynchronises by discarding bytes up to the next 0xA5 sync byte;
    like the C++ emulator, the trailing checksum of payload commands
    is consumed but not verified.
    """
    # Skip to the next sync byte.
    while rx_buffer and rx_buffer[0] != CMD_SYNC_BYTE:
        del rx_buffer[0]

    if len(rx_buffer) < 2:
        return None

    cmd = rx_buffer[1]
    if cmd & CMDFLAG_HAS_PAYLOAD:
        if len(rx_buffer) < 3:
            return None
        payload_size = rx_buffer[2]
        total = 4 + payload_size  # sync + cmd + size + payload + checksum
        if len(rx_buffer) < total:
            return None
        payload = bytes(rx_buffer[3:3 + payload_size])
        del rx_buffer[:total]
        return cmd, payload

    del rx_buffer[:2]
    return cmd, b''


def parse_get_lidar_conf_payload(payload: bytes) -> Optional[Tuple[int, int]]:
    """Parse a GET_LIDAR_CONF payload into (conf_type, mode_id).

    Mode-specific queries append a 2-byte mode_id after the 4-byte
    conf_type; plain queries carry the type only (mode_id -> 0).
    """
    if len(payload) < 4:
        return None
    conf_type = struct.unpack_from('<I', payload, 0)[0]
    mode_id = 0
    if len(payload) >= 6:
        mode_id = struct.unpack_from('<H', payload, 4)[0]
    return conf_type, mode_id


# ---------------------------------------------------------------------------
# Response descriptors and single replies (sensor -> host)
# ---------------------------------------------------------------------------

def build_response_descriptor(data_size: int, ans_type: int,
                              send_mode: int = SEND_MODE_SINGLE) -> bytes:
    """7-byte response descriptor: [A5][5A][size_q30_subtype:4 LE][type]."""
    size_q30_subtype = (data_size & 0x3FFFFFFF) | ((send_mode & 0x3) << 30)
    return struct.pack('<BBIB', ANS_SYNC_BYTE1, ANS_SYNC_BYTE2,
                       size_q30_subtype, ans_type)


def build_device_info_response(info: DeviceInfo) -> bytes:
    """Descriptor + 20-byte payload: model, fw(LE16), hw, serial[16]."""
    serial = bytes(info.serial_number)[:16].ljust(16, b'\x00')
    out = bytearray(build_response_descriptor(20, ANS_TYPE_DEVINFO))
    out.append(info.model & 0xFF)
    out.append(info.firmware_version & 0xFF)
    out.append((info.firmware_version >> 8) & 0xFF)
    out.append(info.hardware_version & 0xFF)
    out.extend(serial)
    return bytes(out)


def build_health_response(status: int = STATUS_OK,
                          error_code: int = 0) -> bytes:
    """Descriptor + 3-byte payload: status, error_code (LE16)."""
    out = bytearray(build_response_descriptor(3, ANS_TYPE_DEVHEALTH))
    out.append(status & 0xFF)
    out.append(error_code & 0xFF)
    out.append((error_code >> 8) & 0xFF)
    return bytes(out)


def build_samplerate_response(t_standard_us: int = DEFAULT_US_PER_SAMPLE,
                              t_express_us: int = DEFAULT_US_PER_SAMPLE
                              ) -> bytes:
    """Descriptor + 4-byte payload: Tstandard, Texpress (us, LE16 each).

    GET_SAMPLERATE (0x59) is the A-series way to query the sample
    period; not implemented by the C++ emulator (which serves the
    S-series GET_LIDAR_CONF path only) but required by A-series hosts.
    """
    out = bytearray(build_response_descriptor(4, ANS_TYPE_SAMPLERATE))
    out.extend(struct.pack('<HH', t_standard_us, t_express_us))
    return bytes(out)


def build_scan_descriptor(ans_type: int = ANS_TYPE_MEASUREMENT) -> bytes:
    """Continuous-mode descriptor announcing the scan stream."""
    data_size = _SCAN_DATA_SIZE.get(ans_type, MEASUREMENT_NODE_BYTES)
    return build_response_descriptor(data_size, ans_type,
                                     SEND_MODE_CONTINUOUS)


def build_lidar_conf_response(conf_type: int, mode_id: int = 0, *,
                              desired_pwm_ref: int = DEFAULT_DESIRED_PWM_REF,
                              desired_rpm: int = DEFAULT_DESIRED_RPM,
                              us_per_sample: int = DEFAULT_US_PER_SAMPLE,
                              max_distance_m: int = DEFAULT_MAX_DISTANCE_M,
                              mac_addr: bytes = DEFAULT_MAC_ADDR,
                              scan_mode_name: str = DEFAULT_SCAN_MODE_NAME
                              ) -> Optional[bytes]:
    """GET_LIDAR_CONF reply: descriptor + echoed type (LE32) + data.

    One "Standard" scan mode (id 0) is advertised, with the same
    values as the C++ emulator. mode_id is accepted but ignored (all
    modes answer with mode-0 values), matching the C++.
    Returns None for unknown conf types.
    """
    del mode_id  # single mode: mode-specific queries all answer mode 0

    payload = bytearray(struct.pack('<I', conf_type))

    if conf_type == LIDAR_CONF_DESIRED_ROT_FREQ:
        payload.extend(struct.pack('<HH', desired_pwm_ref, desired_rpm))
    elif conf_type == LIDAR_CONF_SCAN_MODE_COUNT:
        payload.extend(struct.pack('<H', 1))
    elif conf_type == LIDAR_CONF_SCAN_MODE_TYPICAL:
        payload.extend(struct.pack('<H', 0))
    elif conf_type == LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE:
        payload.extend(struct.pack('<I', us_per_sample * 256))  # q8
    elif conf_type == LIDAR_CONF_SCAN_MODE_MAX_DISTANCE:
        payload.extend(struct.pack('<I', max_distance_m * 256))  # q8
    elif conf_type == LIDAR_CONF_SCAN_MODE_ANS_TYPE:
        payload.append(ANS_TYPE_MEASUREMENT)
    elif conf_type == LIDAR_CONF_LIDAR_MAC_ADDR:
        payload.extend(mac_addr[:6])
    elif conf_type == LIDAR_CONF_SCAN_MODE_NAME:
        payload.extend(scan_mode_name.encode('ascii') + b'\x00')
    else:
        return None

    out = bytearray(build_response_descriptor(
        len(payload), ANS_TYPE_GET_LIDAR_CONF))
    out.extend(payload)
    return bytes(out)


# ---------------------------------------------------------------------------
# Scan stream (sensor -> host)
# ---------------------------------------------------------------------------

def encode_measurement_node(point: ScanPoint, is_sync: bool) -> bytes:
    """Encode one 5-byte standard measurement node (C++ formulas).

    RPLIDAR angles run clockwise (seen from above) while ROS runs
    counter-clockwise, so the angle is negated before conversion to
    the q6 (degrees * 64) wire format.
    """
    # sync_quality: bit0 syncbit, bit1 inverted syncbit, bits 2-7 quality.
    quality6 = (point.quality >> 2) & 0x3F
    sync_quality = quality6 << 2
    sync_quality |= 0x01 if is_sync else 0x02

    # angle_q6_checkbit: bit0 checkbit (always 1), bits 1-15 angle_q6.
    angle_deg = -point.angle * 180.0 / math.pi
    angle_deg %= 360.0
    angle_q6 = int(angle_deg * 64.0) & 0x7FFF
    angle_q6_checkbit = (angle_q6 << 1) | 0x01

    # distance_q2: mm * 4; 0 = invalid; clamped, not wrapped, at uint16.
    distance_q2 = 0
    if point.valid and point.distance > 0:
        distance_q2 = min(int(point.distance * 1000.0 * 4.0),
                          _DISTANCE_Q2_MAX)

    return struct.pack('<BHH', sync_quality, angle_q6_checkbit, distance_q2)


def decode_measurement_node(node: bytes) -> dict:
    """Decode a 5-byte node (host-side view, for tests / smoke scripts).

    Returns a dict with syncbit / syncbit_inverse / quality / checkbit
    plus the decoded angle_deg (wire CW convention) and distance_m.
    """
    if len(node) != MEASUREMENT_NODE_BYTES:
        raise ValueError(f'need {MEASUREMENT_NODE_BYTES} bytes, '
                         f'got {len(node)}')
    sync_quality, angle_q6_checkbit, distance_q2 = struct.unpack('<BHH', node)
    return {
        'syncbit': bool(sync_quality & 0x01),
        'syncbit_inverse': bool(sync_quality & 0x02),
        'quality': sync_quality >> 2,
        'checkbit': bool(angle_q6_checkbit & 0x01),
        'angle_deg': (angle_q6_checkbit >> 1) / 64.0,
        'distance_m': distance_q2 / 4.0 / 1000.0,
        'valid': distance_q2 != 0,
    }


def encode_scan(points: List[ScanPoint]) -> bytes:
    """Encode one revolution: node 0 carries the sync (start) flag."""
    out = bytearray()
    for i, point in enumerate(points):
        out.extend(encode_measurement_node(point, is_sync=(i == 0)))
    return bytes(out)


def dummy_scan(angle_offset_deg: float = 0.0,
               n_points: int = 360) -> List[ScanPoint]:
    """The C++ emulator's fallback pattern when no topic data arrived:

    1-degree resolution, distance 3-7 m varying as 5 + 2*sin(3*angle),
    quality 47.
    """
    points = []
    for i in range(n_points):
        angle = (float(i) + angle_offset_deg) * math.pi / 180.0
        points.append(ScanPoint(
            angle=angle,
            distance=5.0 + 2.0 * math.sin(angle * 3.0),
            quality=47,
            valid=True))
    return points
