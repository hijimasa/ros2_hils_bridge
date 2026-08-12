"""Unit tests for the ID-share Modbus protocol port (no ROS required).

Each test builds or re-parses frames the way orientalmotor_id_share_daemon
(the bus master) does, so a layout change that would break the real
daemon fails here before any serial smoke test does.
"""

import math
import struct

from hils_bridge_actuator_orientalmotor_id_share import oms_motor_spec as spec
from hils_bridge_actuator_orientalmotor_id_share import oms_protocol as proto


# Motors used throughout: one BLV-R 400 W with gear 50 and one AZ 85 mm
# with gear 36 (typical geared-wheel / actuator configurations).
BLVR400 = spec.parse_motor_types('BLVR:400')[0]
AZ85 = spec.parse_motor_types('AZ:85')[0]
BLVR_GEAR = 50.0
AZ_GEAR = 36.0


# ---- CRC ---------------------------------------------------------------

def test_crc16_known_value():
    # Classic Modbus reference vector: 01 03 00 00 00 0A -> CRC C5 CD.
    frame = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x0A])
    crc = proto.crc16(frame)
    assert (crc & 0xFF) == 0xC5
    assert ((crc >> 8) & 0xFF) == 0xCD


def test_crc16_round_trip():
    payload = bytearray(range(48))
    proto.append_crc(payload)
    # A frame with its own CRC appended re-checks to the residual 0.
    assert proto.crc16(bytes(payload)) == 0


# ---- 0x10 setup echo ----------------------------------------------------

def build_setup_frame(addr=0x0F, start=0x1234, qty=2):
    frame = bytearray([addr, proto.FUNC_WRITE_MULTIPLE_REGISTERS])
    frame.extend(struct.pack('>HH', start, qty))
    frame.append(qty * 2)
    frame.extend(b'\x00' * (qty * 2))
    proto.append_crc(frame)
    return bytes(frame)


def test_setup_frame_echo():
    frame = build_setup_frame()
    buf = bytearray(frame)
    extracted = proto.extract_frame(buf)
    assert extracted == frame
    assert not buf
    assert proto.is_setup_frame(extracted)

    echo = proto.build_setup_echo(extracted)
    assert len(echo) == 8
    assert echo[:6] == frame[:6]  # addr + func + start + qty echoed
    assert proto.crc16(echo) == 0  # fresh, valid CRC


# ---- 0x17 request parse --------------------------------------------------

def build_requests():
    blvr_req = proto.MotorRequest(
        mode=proto.MODE_CONTINUOUS_OPERATION_BY_RPM,
        target_velocity=1500,   # BLV-R native: rpm
        limit_torque=800,
        driver_input=proto.DRIVER_INPUT_FREE)
    az_req = proto.MotorRequest(
        mode=proto.MODE_ABSOLUTE_POSITION,
        target_position=-2500,  # AZ native: step (negative exercises sign)
        limit_torque=1000,
        driver_input=proto.DRIVER_INPUT_ALARM_RESET)
    return [blvr_req, az_req]


def test_direct_data_drive_request_parse():
    frame = proto.build_direct_data_drive_request(0x0F, build_requests())
    assert len(frame) == 13 + 2 * proto.REQUEST_BYTES_PER_MOTOR
    assert proto.is_direct_data_drive_frame(frame)

    parsed = proto.parse_direct_data_drive(frame)
    assert parsed is not None
    global_id, requests = parsed
    assert global_id == 0x0F
    assert len(requests) == 2

    assert requests[0].mode == proto.MODE_CONTINUOUS_OPERATION_BY_RPM
    assert requests[0].target_velocity == 1500
    assert requests[0].limit_torque == 800
    assert requests[0].free and not requests[0].alarm_reset

    assert requests[1].mode == proto.MODE_ABSOLUTE_POSITION
    assert requests[1].target_position == -2500
    assert requests[1].limit_torque == 1000
    assert requests[1].alarm_reset and not requests[1].free


def test_direct_data_drive_field_offsets():
    """Pin the raw byte layout of one motor block (offsets/widths/endianness)."""
    req = proto.MotorRequest(
        mode=48, target_position=0x01020304, target_velocity=0x0A0B0C0D,
        limit_torque=0x11121314, driver_input=0x21222324)
    frame = proto.build_direct_data_drive_request(0x0F, [req])
    block = frame[11:11 + proto.REQUEST_BYTES_PER_MOTOR]
    assert block[0:4] == struct.pack('>I', 48)            # mode @ +0
    assert block[4:8] == struct.pack('>I', 0x01020304)    # target_position @ +4
    assert block[8:12] == struct.pack('>I', 0x0A0B0C0D)   # target_velocity @ +8
    assert block[20:24] == struct.pack('>I', 0x11121314)  # limit_torque @ +20
    assert block[28:32] == struct.pack('>I', 0x21222324)  # driver_input @ +28


# ---- 0x17 response layout and unit round trips ---------------------------

def test_direct_data_drive_response_layout():
    blvr_state = proto.MotorResponse(
        free_flag=True, ready_to_return_home_flag=True)
    az_state = proto.MotorResponse(
        stop_flag=True, home_end_flag=True, motor_alarm=0x42,
        communication_error=0x07, current_position=-1234,
        current_velocity=-56, current_torque=900)

    response = proto.build_direct_data_drive_response(
        0x0F, [blvr_state, az_state])

    # [gid][0x17][byte_count][30 bytes/motor...][crc lo][crc hi]
    assert len(response) == 3 + 2 * proto.RESPONSE_BYTES_PER_MOTOR + 2
    assert response[0] == 0x0F
    assert response[1] == proto.FUNC_DIRECT_DATA_DRIVE
    assert response[2] == 2 * proto.RESPONSE_BYTES_PER_MOTOR
    assert proto.crc16(response) == 0  # valid CRC

    def block(i):
        return response[3 + i * proto.RESPONSE_BYTES_PER_MOTOR:
                        3 + (i + 1) * proto.RESPONSE_BYTES_PER_MOTOR]

    b0, b1 = block(0), block(1)

    current_output0 = struct.unpack_from('>I', b0, 0)[0]
    assert current_output0 == (proto.CURRENT_OUTPUT_FREE_BIT
                               | proto.CURRENT_OUTPUT_READY_RETURN_HOME_BIT)

    current_output1 = struct.unpack_from('>I', b1, 0)[0]
    assert current_output1 == proto.CURRENT_OUTPUT_STOP_BIT
    assert struct.unpack_from('>I', b1, 4)[0] == 0x42       # motor_alarm
    assert struct.unpack_from('>I', b1, 8)[0] == 0x07       # communication_error
    assert struct.unpack_from('>i', b1, 12)[0] == -1234     # current_position
    assert struct.unpack_from('>i', b1, 16)[0] == -56       # current_velocity
    assert struct.unpack_from('>i', b1, 20)[0] == 900       # current_torque
    assert struct.unpack_from('>I', b1, 24)[0] == proto.IO_STATUS6_HOME_END_BIT
    assert b1[28:30] == b'\x00\x00'                         # reserved


def test_blvr_conversion_round_trip():
    # Output shaft 2.0 rad/s through gear 50 -> BLV-R native rpm and back.
    out_rad_s = 2.0
    native = spec.output_rad_s_to_native_vel(out_rad_s, BLVR400, BLVR_GEAR)
    # 2.0 rad/s * 50 * 60 / 2pi = 954.9... -> 955 rpm (BLV-R native IS rpm)
    assert native == 955
    back = spec.native_vel_to_output_rad_s(native, BLVR400, BLVR_GEAR)
    assert math.isclose(back, out_rad_s, rel_tol=1e-3)

    # Output shaft 1.0 rad through gear 50 -> steps (36000/rev) and back.
    steps = spec.output_rad_to_motor_step(1.0, BLVR400, BLVR_GEAR)
    assert steps == round(1.0 * BLVR_GEAR * 36000 / (2 * math.pi))
    assert math.isclose(
        spec.motor_step_to_output_rad(steps, BLVR400, BLVR_GEAR),
        1.0, rel_tol=1e-4)

    # 800 permille of the 400 W rated torque (1.27 Nm).
    assert math.isclose(spec.permille_to_nm(800, BLVR400.rated_torque_nm),
                        0.8 * 1.27, rel_tol=1e-9)


def test_az_conversion_round_trip():
    # AZ native velocity is Hz = rpm/60 * 1000 step/rev.
    out_rad_s = 0.5
    native = spec.output_rad_s_to_native_vel(out_rad_s, AZ85, AZ_GEAR)
    motor_rpm = out_rad_s * AZ_GEAR * 60.0 / (2 * math.pi)
    assert native == round(motor_rpm / 60.0 * 1000)
    back = spec.native_vel_to_output_rad_s(native, AZ85, AZ_GEAR)
    assert math.isclose(back, out_rad_s, rel_tol=1e-2)

    steps = spec.output_rad_to_motor_step(2.0, AZ85, AZ_GEAR)
    assert steps == round(2.0 * AZ_GEAR * 1000 / (2 * math.pi))
    assert math.isclose(
        spec.motor_step_to_output_rad(steps, AZ85, AZ_GEAR),
        2.0, rel_tol=1e-3)

    # AZ 85 mm rated (holding) torque is 4.0 Nm.
    assert math.isclose(spec.permille_to_nm(500, AZ85.rated_torque_nm),
                        2.0, rel_tol=1e-9)


def test_position_velocity_survive_response_round_trip():
    """Feedback written in native units survives the wire and converts back."""
    # BLV-R:400 gear 50, joint at 0.75 rad moving 1.2 rad/s.
    blvr_state = proto.MotorResponse(
        current_position=spec.output_rad_to_motor_step(0.75, BLVR400, BLVR_GEAR),
        current_velocity=spec.output_rad_s_to_native_vel(1.2, BLVR400, BLVR_GEAR))
    # AZ:85 gear 36, joint at -0.4 rad moving -0.8 rad/s.
    az_state = proto.MotorResponse(
        current_position=spec.output_rad_to_motor_step(-0.4, AZ85, AZ_GEAR),
        current_velocity=spec.output_rad_s_to_native_vel(-0.8, AZ85, AZ_GEAR))

    response = proto.build_direct_data_drive_response(1, [blvr_state, az_state])
    assert proto.crc16(response) == 0

    def read_block(i):
        base = 3 + i * proto.RESPONSE_BYTES_PER_MOTOR
        pos = struct.unpack_from('>i', response, base + 12)[0]
        vel = struct.unpack_from('>i', response, base + 16)[0]
        return pos, vel

    pos0, vel0 = read_block(0)
    assert math.isclose(
        spec.motor_step_to_output_rad(pos0, BLVR400, BLVR_GEAR),
        0.75, rel_tol=1e-4)
    assert math.isclose(
        spec.native_vel_to_output_rad_s(vel0, BLVR400, BLVR_GEAR),
        1.2, rel_tol=1e-2)

    pos1, vel1 = read_block(1)
    assert math.isclose(
        spec.motor_step_to_output_rad(pos1, AZ85, AZ_GEAR),
        -0.4, rel_tol=1e-3)
    assert math.isclose(
        spec.native_vel_to_output_rad_s(vel1, AZ85, AZ_GEAR),
        -0.8, rel_tol=1e-2)


# ---- resync -------------------------------------------------------------

def test_resync_after_corrupted_leading_byte():
    frame = proto.build_direct_data_drive_request(0x0F, build_requests())
    buf = bytearray(b'\xff')  # garbage byte before a valid frame
    buf.extend(frame)

    extracted = proto.extract_frame(buf)
    assert extracted == frame
    assert not buf


def test_partial_frame_waits_for_more_bytes():
    frame = proto.build_direct_data_drive_request(0x0F, build_requests())
    buf = bytearray(frame[:20])
    assert proto.extract_frame(buf) is None
    assert len(buf) == 20  # untouched: waiting, not resyncing
    buf.extend(frame[20:])
    assert proto.extract_frame(buf) == frame


def test_corrupted_crc_drops_bytes_then_recovers():
    bad = bytearray(proto.build_direct_data_drive_request(0x0F, build_requests()))
    bad[-1] ^= 0xFF  # break the CRC
    good = proto.build_direct_data_drive_request(0x0F, build_requests())
    buf = bytearray(bad)
    buf.extend(good)

    extracted = proto.extract_frame(buf)
    assert extracted == good
    assert not buf
