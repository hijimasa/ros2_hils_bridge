"""OrientalMotor ID-share Modbus-RTU protocol decode/encode (motor/slave side).

Python port of REACT-simulator's orientalmotor_id_share_emulator
oms_protocol.{hpp,cpp}. This is the slave side of the protocol that
orientalmotor_id_share_daemon (master) drives. The register map is
COMMON to the BLV-R and AZ series; every value here is in DRIVER-NATIVE
INTEGER units (steps; BLV-R rpm / AZ Hz; torque 0.1% of rated) exactly
as it appears on the wire. The motor-type-specific rad/rpm/Nm
conversion happens one layer up, in the bridge node (see
oms_motor_spec.py).

The daemon only ever issues two function codes:
    0x10 (Write Multiple Registers) : ID-share / read / write register
        setup and the zero-speed setting. Reply = 8-byte echo.
    0x17 (Read/Write Multiple Registers, "Direct Data Drive") :
        per-cycle command; reply = 30 feedback bytes per motor.

No ROS imports: everything here is unit-testable with plain pytest.
"""

import struct
from dataclasses import dataclass
from typing import List, Optional, Tuple

FUNC_WRITE_MULTIPLE_REGISTERS = 0x10
FUNC_DIRECT_DATA_DRIVE = 0x17

REQUEST_BYTES_PER_MOTOR = 32   # 16 registers * 2 bytes
RESPONSE_BYTES_PER_MOTOR = 30  # 15 registers * 2 bytes

_MIN_FRAME_BYTES = 4    # addr + func + crc(2)
_MAX_FRAME_BYTES = 512  # single-byte length fields bound this

# Operation modes (OrientalmotorIdShareMode values; native, motor-type-independent).
MODE_NOCONTROL = 0
MODE_ABSOLUTE_POSITION = 1
MODE_RELATIVE_POSITION_FROM_TARGET = 2
MODE_RELATIVE_POSITION_FROM_CURRENT = 3
MODE_RETURN_HOME_POSITION = 23
MODE_CONTINUOUS_OPERATION_BY_RPM = 48
MODE_CONTINUOUS_OPERATION_BY_PUSH = 49
MODE_CONTINUOUS_OPERATION_BY_TORQUE = 50
MODE_CONTINUOUS_OPERATION_BY_RPM_CYCLIC = 51

# Driver Input Command bits (request: last 4 bytes of each motor block).
DRIVER_INPUT_ALARM_RESET = 1 << 7
DRIVER_INPUT_FREE = 1 << 6

# Current Output / I/O status bits (response).
CURRENT_OUTPUT_STOP_BIT = 1 << 5
CURRENT_OUTPUT_FREE_BIT = 1 << 6
CURRENT_OUTPUT_READY_RETURN_HOME_BIT = 1 << 10
IO_STATUS6_HOME_END_BIT = 1 << 16


@dataclass
class MotorRequest:
    """A single motor's command, in driver-native integer units."""

    mode: int = 0
    target_position: int = 0   # [step]
    target_velocity: int = 0   # native velocity (BLV-R rpm, AZ Hz)
    limit_torque: int = 0      # 0.1% of rated torque (1000 = 100%)
    driver_input: int = 0      # raw driver input command word
    alarm_reset: bool = False
    free: bool = False


@dataclass
class MotorResponse:
    """A single motor's feedback, in driver-native integer units."""

    current_position: int = 0   # [step]
    current_velocity: int = 0   # native velocity (BLV-R rpm, AZ Hz)
    current_torque: int = 0     # 0.1% of rated torque
    motor_alarm: int = 0
    communication_error: int = 0
    stop_flag: bool = False
    free_flag: bool = False
    ready_to_return_home_flag: bool = False
    home_end_flag: bool = False


def crc16(data: bytes) -> int:
    """Standard Modbus-RTU CRC16 (poly 0xA001, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def append_crc(frame: bytearray) -> None:
    """Append the CRC16 of frame, low byte first (Modbus wire order)."""
    crc = crc16(bytes(frame))
    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)


def _extract_int32_be(data: bytes, offset: int) -> int:
    """Signed 32-bit big-endian read (the wire carries two's complement)."""
    return struct.unpack_from('>i', data, offset)[0]


def _extract_uint32_be(data: bytes, offset: int) -> int:
    return struct.unpack_from('>I', data, offset)[0]


def _append_uint32_be(out: bytearray, value: int) -> None:
    """Append value as big-endian uint32 (negatives as two's complement)."""
    out.extend(struct.pack('>I', value & 0xFFFFFFFF))


def expected_frame_length(buf: bytes) -> Tuple[int, bool]:
    """Total expected frame length from the header.

    Returns:
        (total_length, need_more): total_length is 0 when the function
        code is unknown; need_more is True when the function code is
        known but more header bytes are required to compute the length.
    """
    if len(buf) < 2:
        return 0, True

    func = buf[1]
    if func == FUNC_WRITE_MULTIPLE_REGISTERS:
        # [addr][0x10][start:2][qty:2][byte_count][data..][crc:2]
        if len(buf) < 7:
            return 0, True
        return 9 + buf[6], False
    if func == FUNC_DIRECT_DATA_DRIVE:
        # [gid][0x17][r_addr:2][r_len:2][w_addr:2][w_len:2][w_byte_count][data..][crc:2]
        if len(buf) < 11:
            return 0, True
        return 13 + buf[10], False

    return 0, False  # unknown function code


def extract_frame(rx_buffer: bytearray) -> Optional[bytes]:
    """Pop one complete, CRC-valid Modbus frame off the front of rx_buffer.

    Resynchronises by discarding leading bytes that cannot begin a known
    frame (unexpected function code, or a frame whose CRC fails).
    Returns None (leaving rx_buffer intact) when more bytes are needed
    to complete a frame.
    """
    while rx_buffer:
        total, need_more = expected_frame_length(bytes(rx_buffer))

        if need_more:
            return None  # wait for more header bytes

        if total < _MIN_FRAME_BYTES or total > _MAX_FRAME_BYTES:
            del rx_buffer[0]  # unknown / implausible: resync
            continue

        if len(rx_buffer) < total:
            return None  # wait for the rest of the frame

        crc_calc = crc16(bytes(rx_buffer[:total - 2]))
        if (crc_calc & 0xFF) == rx_buffer[total - 2] and \
                ((crc_calc >> 8) & 0xFF) == rx_buffer[total - 1]:
            frame = bytes(rx_buffer[:total])
            del rx_buffer[:total]
            return frame

        del rx_buffer[0]  # bad CRC: drop one byte and resync

    return None


def is_setup_frame(frame: bytes) -> bool:
    return len(frame) >= 2 and frame[1] == FUNC_WRITE_MULTIPLE_REGISTERS


def is_direct_data_drive_frame(frame: bytes) -> bool:
    return len(frame) >= 2 and frame[1] == FUNC_DIRECT_DATA_DRIVE


def build_setup_echo(frame: bytes) -> Optional[bytes]:
    """Build the 8-byte echo reply to a 0x10 setup frame.

    Standard Write-Multiple-Registers reply: echo [addr][func][start:2]
    [qty:2] plus a fresh CRC. This satisfies every 0x10 setup the
    daemon sends (zero-speed, ID-share, read-register, write-register),
    all of which check the echoed bytes. Returns None if too short.
    """
    if len(frame) < 6:
        return None
    response = bytearray(frame[:6])
    append_crc(response)
    return bytes(response)


def parse_direct_data_drive(
        frame: bytes) -> Optional[Tuple[int, List[MotorRequest]]]:
    """Parse a 0x17 Direct-Data-Drive command frame.

    Returns:
        (global_id, requests) with one MotorRequest per motor present
        in the frame, or None if malformed. global_id is byte 0
        (echoed back in the reply).
    """
    if len(frame) < 13:
        return None

    global_id = frame[0]

    write_byte_count = frame[10]
    motor_count = write_byte_count // REQUEST_BYTES_PER_MOTOR
    expected_size = 11 + write_byte_count + 2
    if motor_count == 0 or len(frame) != expected_size:
        return None

    requests = []
    offset = 11
    for _ in range(motor_count):
        req = MotorRequest()
        req.mode = _extract_int32_be(frame, offset)
        req.target_position = _extract_int32_be(frame, offset + 4)
        req.target_velocity = _extract_int32_be(frame, offset + 8)
        # bytes 12-15 acceleration time, 16-19 deceleration time: ignored.
        req.limit_torque = _extract_int32_be(frame, offset + 20)
        # bytes 24-27 lifetime / velocity unit: ignored.
        req.driver_input = _extract_uint32_be(frame, offset + 28)
        req.alarm_reset = bool(req.driver_input & DRIVER_INPUT_ALARM_RESET)
        req.free = bool(req.driver_input & DRIVER_INPUT_FREE)
        requests.append(req)
        offset += REQUEST_BYTES_PER_MOTOR

    return global_id, requests


def build_direct_data_drive_response(
        global_id: int, responses: List[MotorResponse]) -> bytes:
    """Build a 0x17 feedback reply (30 bytes/motor + CRC), echoing global_id."""
    out = bytearray()
    out.append(global_id)
    out.append(FUNC_DIRECT_DATA_DRIVE)
    out.append((RESPONSE_BYTES_PER_MOTOR * len(responses)) & 0xFF)

    for state in responses:
        current_output = 0
        if state.stop_flag:
            current_output |= CURRENT_OUTPUT_STOP_BIT
        if state.free_flag:
            current_output |= CURRENT_OUTPUT_FREE_BIT
        if state.ready_to_return_home_flag:
            current_output |= CURRENT_OUTPUT_READY_RETURN_HOME_BIT

        io_status_6 = 0
        if state.home_end_flag:
            io_status_6 |= IO_STATUS6_HOME_END_BIT

        _append_uint32_be(out, current_output)
        _append_uint32_be(out, state.motor_alarm)
        _append_uint32_be(out, state.communication_error)
        _append_uint32_be(out, state.current_position)
        _append_uint32_be(out, state.current_velocity)
        _append_uint32_be(out, state.current_torque)
        _append_uint32_be(out, io_status_6)
        out.append(0)  # reserved
        out.append(0)

    append_crc(out)
    return bytes(out)


def is_position_mode(mode: int) -> bool:
    return mode in (MODE_ABSOLUTE_POSITION,
                    MODE_RELATIVE_POSITION_FROM_TARGET,
                    MODE_RELATIVE_POSITION_FROM_CURRENT,
                    MODE_RETURN_HOME_POSITION)


def is_velocity_mode(mode: int) -> bool:
    return mode in (MODE_CONTINUOUS_OPERATION_BY_RPM,
                    MODE_CONTINUOUS_OPERATION_BY_RPM_CYCLIC)


def is_effort_mode(mode: int) -> bool:
    return mode == MODE_CONTINUOUS_OPERATION_BY_TORQUE


def build_direct_data_drive_request(
        global_id: int, requests: List[MotorRequest], *,
        read_addr: int = 0x0000, write_addr: int = 0x0000) -> bytes:
    """Build a master-side 0x17 Direct-Data-Drive request frame.

    Not used by the bridge node itself (it is the slave); provided for
    tests and master-side smoke scripts so they can poll this emulator
    with well-formed frames.
    """
    write_byte_count = REQUEST_BYTES_PER_MOTOR * len(requests)
    out = bytearray()
    out.append(global_id)
    out.append(FUNC_DIRECT_DATA_DRIVE)
    # read: 15 registers per motor, write: 16 registers per motor.
    out.extend(struct.pack('>HH', read_addr,
                           (RESPONSE_BYTES_PER_MOTOR // 2) * len(requests)))
    out.extend(struct.pack('>HH', write_addr,
                           (REQUEST_BYTES_PER_MOTOR // 2) * len(requests)))
    out.append(write_byte_count & 0xFF)
    for req in requests:
        _append_uint32_be(out, req.mode)
        _append_uint32_be(out, req.target_position)
        _append_uint32_be(out, req.target_velocity)
        _append_uint32_be(out, 0)  # acceleration time (ignored by the slave)
        _append_uint32_be(out, 0)  # deceleration time (ignored by the slave)
        _append_uint32_be(out, req.limit_torque)
        _append_uint32_be(out, 0)  # lifetime / velocity unit (ignored)
        _append_uint32_be(out, req.driver_input)
    append_crc(out)
    return bytes(out)
