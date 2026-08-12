"""Unit tests for the VSSP 2.1 encoder (no ROS, no sockets required).

Each test re-parses the encoder's output the way urg3d_library does
(Urg3dSensor.cpp), so a layout change that would break the real driver
fails here first.
"""

import math
import struct

import numpy as np

from hils_bridge_lidar_hokuyo_yvt35lx import vssp_protocol as vp


def parse_header(buf):
    mark, ptype, status, hlen, length, _rt, _st = struct.unpack_from(
        '<4s4s4sHHII', buf, 0)
    assert mark == b'VSSP'
    assert hlen == vp.HEADER_LEN
    assert length == len(buf)
    return ptype[:3].decode(), status[:3].decode(), length, buf[24:length]


def hex_chars_to_u32(data, begin, length):
    """urg3d_library's hexCharsToU32() over the response body."""
    return int(data[begin:begin + length].decode(), 16)


def read_angle_table(body, total_length, start_pos=9):
    """Mirror the library's `for (i = 9; i + 4 <= len; i += 5)` loop."""
    values = []
    i = start_pos
    while i + vp.HEADER_LEN - 20 <= total_length - vp.HEADER_LEN:
        if i + 4 > total_length - vp.HEADER_LEN:
            break
        values.append(hex_chars_to_u32(body, i, 4))
        i += 5
    return values


def test_ver_response_starts_with_vend():
    pkt = vp.build_ver_response('HOKUYO AUTOMATIC CO.,LTD.', 'YVT-35LX',
                                'H0000001', '1.2.0-hils', 'VSSP2.1', 1234)
    ptype, status, _length, body = parse_header(pkt)
    assert (ptype, status) == ('VER', vp.STATUS_OK)
    # highBlockingGetSensorVersion matches on the "vend" dataHead.
    assert body.startswith(b'vend:')
    assert b'prot:VSSP2.1\n' in body


def test_command_response_echoes_command():
    # highBlockingCommon accepts a response only when the body starts
    # with the command it sent.
    for command, payload in (('GET:_itl', '0,01\n'),
                             ('GET:_itv', '0,03\n'),
                             ('DAT:ri=1', ''),
                             ('SET:_itl=0,02', ''),
                             ('RST', '')):
        pkt = vp.build_response(command, vp.STATUS_OK, payload, 0)
        ptype, status, _length, body = parse_header(pkt)
        assert ptype == command[:3]
        assert status == vp.STATUS_OK
        assert body.startswith(command.encode() + b'\n')


def test_float_timestamps_are_accepted():
    # The node's sensor clock is a float; the encoder must truncate
    # rather than raise (regression: TypeError on `float & int`).
    pkt = vp.build_response('GET:stat', vp.STATUS_OK, '', 1234.567)
    _t, _s, _length, _body = parse_header(pkt)


def test_motor_angle_table_spans_full_line():
    table = vp.motor_ratio_table()
    pkt = vp.encode_angle_table('GET:tblh', table, 0)
    _t, _s, length, body = parse_header(pkt)
    ratios = [v / 65535.0 for v in read_angle_table(body, length)]
    assert len(ratios) == vp.SPOT_COUNT
    assert ratios[0] == 0.0
    assert abs(ratios[-1] - 1.0) < 1e-9
    assert all(b >= a for a, b in zip(ratios, ratios[1:]))


def test_vertical_angle_table_decodes_to_requested_fov():
    pkt = vp.encode_angle_table(
        'GET:tv00', vp.vertical_angle_table(-20.0, 20.0), 0)
    _t, _s, length, body = parse_header(pkt)
    degrees = []
    for raw in read_angle_table(body, length):
        rad = raw * 2.0 * math.pi / 65535.0
        if rad > math.pi:      # the library's wrap for negative angles
            rad -= 2.0 * math.pi
        degrees.append(math.degrees(rad))
    assert len(degrees) == vp.SPOT_COUNT
    assert abs(degrees[0] + 20.0) < 0.02
    assert abs(degrees[-1] - 20.0) < 0.02


def parse_range_packet(pkt):
    ptype, _status, _length, body = parse_header(pkt)
    header_length = struct.unpack_from('<H', body, 0)[0]
    buf = body[2:header_length]
    frame, motor_field = struct.unpack_from('<BB', buf, 12)
    line, spot = struct.unpack_from('<HH', buf, 14)
    rem_field, rem_interlace = struct.unpack_from('<BB', buf, 18)

    offset = header_length
    index_length, nspots = struct.unpack_from('<HH', body, offset)
    offset += 4
    index = struct.unpack_from(f'<{nspots + 1}H', body, offset)
    offset += (nspots + 1) * 2
    if nspots % 2 == 0:
        offset += 2  # reserve
    n_echo = index[nspots]
    if ptype == '_ri':
        raw = struct.unpack_from(f'<{n_echo * 2}H', body, offset)
        echoes = [(raw[2 * i], raw[2 * i + 1]) for i in range(n_echo)]
        offset += n_echo * 4
    else:
        echoes = [(v, 0) for v in
                  struct.unpack_from(f'<{n_echo}H', body, offset)]
        offset += n_echo * 2

    assert header_length == vp.RANGE_HEADER_LEN
    assert index_length == 4 + (nspots + 1) * 2 + (2 if nspots % 2 == 0 else 0)
    assert offset == len(body), 'trailing bytes in packet body'
    return dict(ptype=ptype, frame=frame, motor_field=motor_field, line=line,
                spot=spot, rem_field=rem_field, rem_interlace=rem_interlace,
                nspots=nspots, index=index, echoes=echoes)


def _one_line(with_intensity=True, **overrides):
    ranges = np.zeros(vp.SPOT_COUNT, dtype=np.uint16)
    intensities = np.zeros(vp.SPOT_COUNT, dtype=np.uint16)
    ranges[3], intensities[3] = 1000, 42
    ranges[vp.SPOT_COUNT - 1], intensities[vp.SPOT_COUNT - 1] = 9999, 7
    geometry = vp.ScanGeometry()
    kwargs = dict(with_intensity=with_intensity, frame=5, motor_field=0,
                  rem_field=0, rem_interlace=1, line=7, ts_head_ms=100,
                  ts_tail_ms=102, head_ratio=geometry.head_ratios[7],
                  tail_ratio=geometry.tail_ratios[7], ranges_mm=ranges,
                  intensities=intensities, ts_ms=102)
    kwargs.update(overrides)
    return vp.build_range_packet(**kwargs)


def test_ri_packet_indexes_only_spots_with_echoes():
    parsed = parse_range_packet(_one_line())
    assert parsed['ptype'] == '_ri'
    assert (parsed['frame'], parsed['line'], parsed['spot']) == (5, 7, 0)
    assert parsed['nspots'] == vp.SPOT_COUNT
    # Cumulative index: spot 3 and the last spot each carry one echo.
    assert parsed['index'][3] == 0 and parsed['index'][4] == 1
    assert parsed['index'][vp.SPOT_COUNT] == 2
    assert parsed['echoes'] == [(1000, 42), (9999, 7)]


def test_echo_count_per_spot_stays_within_sensor_limit():
    parsed = parse_range_packet(_one_line())
    index = parsed['index']
    for spot in range(vp.SPOT_COUNT):
        assert 0 <= index[spot + 1] - index[spot] <= 4


def test_ro_packet_carries_range_only():
    parsed = parse_range_packet(_one_line(with_intensity=False))
    assert parsed['ptype'] == '_ro'
    assert parsed['echoes'] == [(1000, 0), (9999, 0)]


def test_ax_packet_scales_match_driver_constants():
    record = vp.aux_record_raw(accel_g=(0.0, 0.0, 1.0), temp_c=25.0)
    pkt = vp.build_ax_packet(5000, [record] * 10, 10)
    ptype, _status, length, body = parse_header(pkt)
    header_length, _ts, bitfield, count, data_ms = struct.unpack_from(
        '<HIIBB', body, 0)
    assert ptype == '_ax'
    assert (header_length, count, data_ms) == (12, 10, 10)
    assert bitfield == vp.AX_BITFIELD
    n_values = (length - vp.HEADER_LEN - 12) // 4
    assert n_values == 10 * vp.AX_VALUES_PER_RECORD
    values = struct.unpack_from(f'<{n_values}i', body, 12)
    assert abs(values[5] - 0x7FFF / 4) < 2      # accel Z = 1 g
    temperature = values[9] / vp.TEMPERATURE_FACTOR + vp.TEMPERATURE_OFFSET
    assert abs(temperature - 25.0) < 0.1


def test_wall_survives_the_scan_grid_round_trip():
    """A 10 m wall must reconstruct near 10 m through the library's math."""
    geometry = vp.ScanGeometry()
    ys = np.arange(-3.0, 3.01, 0.1)
    zs = np.arange(-1.5, 1.51, 0.1)
    yy, zz = np.meshgrid(ys, zs)
    y, z = yy.ravel(), zz.ravel()
    x = np.full(y.size, 10.0)
    range_grid, intensity_grid = geometry.bin_pointcloud(
        x, y, z, np.full(y.size, 100.0))
    assert (range_grid > 0).sum() > 150
    assert intensity_grid[range_grid > 0].min() == 100

    motor = [v / 65535.0 for v in vp.motor_ratio_table()]
    vertical = []
    for raw in vp.vertical_angle_table(geometry.v_min_deg, geometry.v_max_deg):
        rad = raw * 2.0 * math.pi / 65535.0
        vertical.append(rad - 2.0 * math.pi if rad > math.pi else rad)

    for line in range(vp.LINE_COUNT):
        head = geometry.head_ratios[line]
        tail = geometry.tail_ratios[line]
        for spot in range(vp.SPOT_COUNT):
            if range_grid[line][spot] == 0:
                continue
            motor_rad = ((head + (tail - head) * motor[spot])
                         / 65535.0 * 2.0 * math.pi)
            distance = range_grid[line][spot] / 1000.0
            px = distance * math.cos(vertical[spot]) * math.cos(motor_rad)
            py = distance * math.cos(vertical[spot]) * math.sin(motor_rad)
            pz = distance * math.sin(vertical[spot])
            assert abs(px - 10.0) < 0.9, (line, spot, px)
            assert -3.6 < py < 3.6
            assert -2.0 < pz < 2.0


def test_points_outside_the_field_of_view_are_dropped():
    geometry = vp.ScanGeometry(h_fov_deg=210.0, v_min_deg=-20.0,
                               v_max_deg=20.0)
    # Straight up (elevation 90 deg) and behind the blind sector.
    x = np.array([0.0, -10.0])
    y = np.array([0.0, -0.1])
    z = np.array([5.0, 0.0])
    range_grid, _ = geometry.bin_pointcloud(x, y, z, np.zeros(2))
    assert (range_grid > 0).sum() == 0


def _grid(value_mm=5000, intensity=100):
    ranges = np.full((vp.LINE_COUNT, vp.SPOT_COUNT), value_mm, dtype=np.uint16)
    intensities = np.full((vp.LINE_COUNT, vp.SPOT_COUNT), intensity,
                          dtype=np.uint16)
    return ranges, intensities


def test_noise_is_off_by_default_so_it_cannot_double_up_with_a_simulator():
    noise = vp.RangeNoise()
    assert not noise.enabled
    ranges, intensities = _grid()
    out_r, out_i = noise.apply(ranges, intensities)
    assert np.array_equal(out_r, ranges)
    assert np.array_equal(out_i, intensities)


def test_range_noise_matches_the_requested_sigma():
    noise = vp.RangeNoise(sigma_m=0.02, seed=7)
    out_r, _ = noise.apply(*_grid(5000))
    deviations = out_r.astype(np.float64) - 5000.0
    assert abs(deviations.mean()) < 3.0          # unbiased, in mm
    assert 15.0 < deviations.std() < 25.0        # ~20 mm requested


def test_same_seed_reproduces_the_same_measurements():
    a, _ = vp.RangeNoise(sigma_m=0.02, seed=42).apply(*_grid())
    b, _ = vp.RangeNoise(sigma_m=0.02, seed=42).apply(*_grid())
    c, _ = vp.RangeNoise(sigma_m=0.02, seed=43).apply(*_grid())
    assert np.array_equal(a, b)
    assert not np.array_equal(a, c)


def test_dropout_removes_returns_and_their_intensity():
    noise = vp.RangeNoise(dropout_probability=0.25, seed=3)
    out_r, out_i = noise.apply(*_grid())
    dropped = out_r == 0
    total = out_r.size
    assert 0.20 * total < dropped.sum() < 0.30 * total
    # A spot with no echo must not carry an intensity either.
    assert np.all(out_i[dropped] == 0)
    assert np.all(out_i[~dropped] == 100)


def test_noise_never_reports_outside_the_measurement_range():
    # A spot at the far limit, nudged by a huge sigma, must drop out
    # rather than wrap around or exceed the datasheet range.
    noise = vp.RangeNoise(sigma_m=5.0, seed=11)
    out_r, _ = noise.apply(*_grid(int(vp.MAX_RANGE_M * 1000)))
    reported = out_r[out_r > 0]
    assert reported.size > 0
    assert reported.min() >= vp.MIN_RANGE_M * 1000
    assert reported.max() <= vp.MAX_RANGE_M * 1000


def test_noise_leaves_spots_without_echo_empty():
    ranges, intensities = _grid()
    ranges[0, :] = 0
    intensities[0, :] = 0
    out_r, _ = vp.RangeNoise(sigma_m=0.02, seed=5).apply(ranges, intensities)
    assert np.all(out_r[0, :] == 0)


def test_points_outside_the_measurement_range_are_dropped():
    geometry = vp.ScanGeometry()
    # Closer than 0.3 m and farther than 35 m: the real sensor reports
    # neither, so neither may appear in the packets.
    x = np.array([0.1, 40.0, 10.0])
    y = np.zeros(3)
    z = np.zeros(3)
    range_grid, _ = geometry.bin_pointcloud(x, y, z, np.zeros(3))
    assert (range_grid > 0).sum() == 1
    assert range_grid.max() == 10000
