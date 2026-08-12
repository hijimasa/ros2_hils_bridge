"""Unit tests for the SLAMTEC RPLIDAR protocol port (no ROS required).

Each test builds or re-parses frames the way the SLAMTEC SDK /
rplidar_ros driver does, so a layout change that would break the real
driver fails here before any serial smoke test does. Expected byte
values are computed independently with the C++ emulator's formulas
(rplidar_protocol.cpp).
"""

import math
import struct

from hils_bridge_lidar_slamtec_rplidar_a import rplidar_protocol as proto


# ---- request build / parse ------------------------------------------------

def test_build_command_no_payload():
    assert proto.build_command(proto.CMD_GET_DEVICE_INFO) == b'\xa5\x50'
    assert proto.build_command(proto.CMD_SCAN) == b'\xa5\x20'
    assert proto.build_command(proto.CMD_STOP) == b'\xa5\x25'


def test_build_command_payload_checksum():
    payload = struct.pack('<I', proto.LIDAR_CONF_SCAN_MODE_COUNT)
    frame = proto.build_command(proto.CMD_GET_LIDAR_CONF, payload)
    # [A5][84][04][70 00 00 00][checksum]
    assert frame[:3] == bytes([0xA5, 0x84, 0x04])
    assert frame[3:7] == payload
    # SLAMTEC checksum: XOR of all preceding bytes -> whole frame XORs to 0.
    assert proto.xor_checksum(frame) == 0
    assert frame[-1] == 0xA5 ^ 0x84 ^ 0x04 ^ 0x70


def test_extract_simple_command():
    buf = bytearray(proto.build_command(proto.CMD_GET_DEVICE_HEALTH))
    cmd, payload = proto.extract_command(buf)
    assert cmd == proto.CMD_GET_DEVICE_HEALTH
    assert payload == b''
    assert not buf


def test_extract_payload_command():
    payload = struct.pack('<IH', proto.LIDAR_CONF_SCAN_MODE_NAME, 0)
    buf = bytearray(proto.build_command(proto.CMD_GET_LIDAR_CONF, payload))
    cmd, extracted = proto.extract_command(buf)
    assert cmd == proto.CMD_GET_LIDAR_CONF
    assert extracted == payload
    assert not buf  # checksum consumed too
    assert proto.parse_get_lidar_conf_payload(extracted) == \
        (proto.LIDAR_CONF_SCAN_MODE_NAME, 0)


def test_extract_resyncs_past_garbage_and_handles_back_to_back():
    buf = bytearray(b'\x00\xff')  # garbage before the sync byte
    buf.extend(proto.build_command(proto.CMD_GET_DEVICE_INFO))
    buf.extend(proto.build_command(proto.CMD_SCAN))
    assert proto.extract_command(buf) == (proto.CMD_GET_DEVICE_INFO, b'')
    assert proto.extract_command(buf) == (proto.CMD_SCAN, b'')
    assert proto.extract_command(buf) is None
    assert not buf


def test_extract_partial_command_waits():
    payload = struct.pack('<I', proto.LIDAR_CONF_SCAN_MODE_COUNT)
    frame = proto.build_command(proto.CMD_GET_LIDAR_CONF, payload)
    buf = bytearray(frame[:5])
    assert proto.extract_command(buf) is None
    assert len(buf) == 5  # untouched: waiting, not resyncing
    buf.extend(frame[5:])
    assert proto.extract_command(buf) == (proto.CMD_GET_LIDAR_CONF, payload)


# ---- response descriptors and single replies ------------------------------

def test_device_info_response_layout():
    info = proto.DeviceInfo(model=0x61, firmware_version=0x0118,
                            hardware_version=0x07,
                            serial_number=b'EMULATOR00000001')
    response = proto.build_device_info_response(info)
    assert len(response) == 7 + 20
    # Descriptor: [A5][5A][14 00 00 00 = 20, single mode][04]
    assert response[:7] == bytes([0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04])
    assert response[7] == 0x61                      # model
    assert response[8:10] == bytes([0x18, 0x01])    # firmware LE16
    assert response[10] == 0x07                     # hardware
    assert response[11:27] == b'EMULATOR00000001'   # serial, 16 bytes


def test_device_info_serial_padded_to_16():
    info = proto.DeviceInfo(serial_number=b'SHORT')
    response = proto.build_device_info_response(info)
    assert len(response) == 27
    assert response[11:27] == b'SHORT' + b'\x00' * 11


def test_health_response_layout():
    response = proto.build_health_response(proto.STATUS_WARNING, 0x0020)
    assert len(response) == 7 + 3
    assert response[:7] == bytes([0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06])
    assert response[7] == proto.STATUS_WARNING
    assert response[8:10] == bytes([0x20, 0x00])    # error code LE16


def test_samplerate_response_layout():
    response = proto.build_samplerate_response(130, 130)
    assert len(response) == 7 + 4
    assert response[:7] == bytes([0xA5, 0x5A, 0x04, 0x00, 0x00, 0x00, 0x15])
    assert struct.unpack_from('<HH', response, 7) == (130, 130)


def test_scan_descriptor_continuous_mode():
    descriptor = proto.build_scan_descriptor(proto.ANS_TYPE_MEASUREMENT)
    assert len(descriptor) == 7
    assert descriptor[:2] == bytes([0xA5, 0x5A])
    size_q30 = struct.unpack_from('<I', descriptor, 2)[0]
    assert size_q30 & 0x3FFFFFFF == 5               # 5-byte nodes
    assert size_q30 >> 30 == proto.SEND_MODE_CONTINUOUS
    assert descriptor[6] == proto.ANS_TYPE_MEASUREMENT
    # Exact bytes the C++ emits: A5 5A 05 00 00 40 81.
    assert descriptor == bytes([0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81])


def test_express_hq_descriptor_sizes():
    for ans_type, size in ((proto.ANS_TYPE_MEASUREMENT_CAPSULED, 84),
                           (proto.ANS_TYPE_MEASUREMENT_HQ, 781)):
        descriptor = proto.build_scan_descriptor(ans_type)
        size_q30 = struct.unpack_from('<I', descriptor, 2)[0]
        assert size_q30 & 0x3FFFFFFF == size
        assert size_q30 >> 30 == proto.SEND_MODE_CONTINUOUS
        assert descriptor[6] == ans_type


# ---- GET_LIDAR_CONF replies ------------------------------------------------

def _conf_payload(response):
    """Split a LIDAR_CONF response into (descriptor, echoed_type, data)."""
    descriptor, body = response[:7], response[7:]
    assert descriptor[:2] == bytes([0xA5, 0x5A])
    assert descriptor[6] == proto.ANS_TYPE_GET_LIDAR_CONF
    size_q30 = struct.unpack_from('<I', descriptor, 2)[0]
    assert size_q30 & 0x3FFFFFFF == len(body)
    assert size_q30 >> 30 == proto.SEND_MODE_SINGLE
    return struct.unpack_from('<I', body, 0)[0], body[4:]


def test_lidar_conf_scan_mode_count_typical_ans_type():
    echoed, data = _conf_payload(proto.build_lidar_conf_response(
        proto.LIDAR_CONF_SCAN_MODE_COUNT))
    assert echoed == proto.LIDAR_CONF_SCAN_MODE_COUNT
    assert struct.unpack('<H', data)[0] == 1        # one mode

    echoed, data = _conf_payload(proto.build_lidar_conf_response(
        proto.LIDAR_CONF_SCAN_MODE_TYPICAL))
    assert echoed == proto.LIDAR_CONF_SCAN_MODE_TYPICAL
    assert struct.unpack('<H', data)[0] == 0        # typical = mode 0

    echoed, data = _conf_payload(proto.build_lidar_conf_response(
        proto.LIDAR_CONF_SCAN_MODE_ANS_TYPE))
    assert data == bytes([proto.ANS_TYPE_MEASUREMENT])


def test_lidar_conf_q8_values_and_rot_freq():
    _, data = _conf_payload(proto.build_lidar_conf_response(
        proto.LIDAR_CONF_SCAN_MODE_US_PER_SAMPLE))
    assert struct.unpack('<I', data)[0] == 130 * 256    # us in q8

    _, data = _conf_payload(proto.build_lidar_conf_response(
        proto.LIDAR_CONF_SCAN_MODE_MAX_DISTANCE))
    assert struct.unpack('<I', data)[0] == 30 * 256     # metres in q8

    _, data = _conf_payload(proto.build_lidar_conf_response(
        proto.LIDAR_CONF_DESIRED_ROT_FREQ))
    assert struct.unpack('<HH', data) == (600, 600)     # pwm_ref, rpm


def test_lidar_conf_name_mac_and_unknown():
    _, data = _conf_payload(proto.build_lidar_conf_response(
        proto.LIDAR_CONF_SCAN_MODE_NAME))
    assert data == b'Standard\x00'                  # null-terminated

    _, data = _conf_payload(proto.build_lidar_conf_response(
        proto.LIDAR_CONF_LIDAR_MAC_ADDR))
    assert data == bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55])

    assert proto.build_lidar_conf_response(0xDEADBEEF) is None


# ---- measurement nodes -----------------------------------------------------

def _cpp_node(angle_rad, distance_m, quality, valid, is_sync):
    """Reference 5-byte encoding, transcribed from the C++
    generateStandardMeasurementNode() line by line."""
    sync_quality = ((quality >> 2) << 2) & 0xFF
    sync_quality |= 0x01 if is_sync else 0x02
    angle_deg = -angle_rad * 180.0 / math.pi
    while angle_deg < 0:
        angle_deg += 360.0
    while angle_deg >= 360.0:
        angle_deg -= 360.0
    angle_q6 = int(angle_deg * 64.0) & 0x7FFF
    word = ((angle_q6 << 1) | 0x01) & 0xFFFF
    distance_q2 = 0
    if valid and distance_m > 0:
        distance_q2 = int(min(distance_m * 1000.0 * 4.0, 65534.0))
    return bytes([sync_quality, word & 0xFF, (word >> 8) & 0xFF,
                  distance_q2 & 0xFF, (distance_q2 >> 8) & 0xFF])


def test_node_start_flag_bits():
    point = proto.ScanPoint(angle=0.0, distance=1.0, quality=47, valid=True)
    sync_node = proto.encode_measurement_node(point, is_sync=True)
    normal_node = proto.encode_measurement_node(point, is_sync=False)
    # bit0 = syncbit, bit1 = inverted syncbit: exactly one is set.
    assert sync_node[0] & 0x03 == 0x01
    assert normal_node[0] & 0x03 == 0x02
    # quality 47 -> 6-bit field 11 in bits 2-7.
    assert sync_node[0] >> 2 == 47 >> 2
    # checkbit (bit0 of the angle word) always set.
    assert sync_node[1] & 0x01 == 0x01
    assert normal_node[1] & 0x01 == 0x01


def test_node_matches_cpp_formula_over_sweep():
    angles = [-math.pi, -2.0, -0.5, 0.0, 0.001, 0.5, 1.0, 2.5,
              math.pi, 4.0, 7.0]
    distances = [0.02, 0.15, 1.0, 5.0, 12.34, 16.0, 30.0]
    quality = 200
    for i, angle in enumerate(angles):
        distance = distances[i % len(distances)]
        point = proto.ScanPoint(angle=angle, distance=distance,
                                quality=quality, valid=True)
        assert proto.encode_measurement_node(point, is_sync=(i == 0)) == \
            _cpp_node(angle, distance, quality, True, i == 0)


def test_node_invalid_and_clamped_distance():
    invalid = proto.ScanPoint(angle=1.0, distance=5.0, quality=0, valid=False)
    node = proto.encode_measurement_node(invalid, is_sync=False)
    assert node[3:5] == b'\x00\x00'      # invalid -> distance_q2 = 0

    # 30 m exceeds the uint16 q2 range: clamped to 65534, not wrapped.
    far = proto.ScanPoint(angle=1.0, distance=30.0, quality=47, valid=True)
    node = proto.encode_measurement_node(far, is_sync=False)
    assert struct.unpack_from('<H', node, 3)[0] == 65534


def test_node_round_trip_angle_distance():
    for angle in [0.0, 0.4, -0.4, 1.7, -3.0, 3.1]:
        for distance in [0.05, 0.7, 2.5, 9.99]:
            point = proto.ScanPoint(angle=angle, distance=distance,
                                    quality=47, valid=True)
            decoded = proto.decode_measurement_node(
                proto.encode_measurement_node(point, is_sync=False))
            assert decoded['valid'] and decoded['checkbit']
            # Wire angle is CW: negate + wrap to compare with ROS angle.
            expected_deg = (-angle * 180.0 / math.pi) % 360.0
            assert abs(decoded['angle_deg'] - expected_deg) <= 1.0 / 64.0
            # q2 quantisation: 0.25 mm.
            assert abs(decoded['distance_m'] - distance) <= 0.00025
            assert decoded['quality'] == 47 >> 2


def test_encode_scan_sync_only_on_first_node():
    points = proto.dummy_scan()
    stream = proto.encode_scan(points)
    assert len(stream) == len(points) * proto.MEASUREMENT_NODE_BYTES
    for i in range(len(points)):
        byte0 = stream[i * proto.MEASUREMENT_NODE_BYTES]
        if i == 0:
            assert byte0 & 0x03 == 0x01     # sync
        else:
            assert byte0 & 0x03 == 0x02     # not sync
    # Dummy pattern: every node valid, quality 47.
    decoded = proto.decode_measurement_node(stream[:5])
    assert decoded['valid'] and decoded['quality'] == 47 >> 2
