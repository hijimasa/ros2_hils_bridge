"""
Hokuyo VSSP 2.1 protocol encoder for the YVT-35LX emulator.

Pure functions (no ROS / no sockets) so the packet layout is
unit-testable. The layout is written against the actual consumer,
urg3d_library (Urg3dSensor.cpp / Urg3d_t.h), not against a paraphrase
of the spec:

  VSSP common header (24 bytes, little-endian):
      char  mark[4]      = "VSSP"
      char  type[4]      = 3 chars + LF  (e.g. "VER\\n", "_ri\\n")
      char  status[4]    = 3 chars + LF  ("000\\n" = OK)
      u16   header_length = 24
      u16   length        = header + body total bytes
      u32   received_time_ms, u32 send_time_ms  (sensor clock)

  Command responses are ASCII bodies. GET/SET/DAT/RST bodies must echo
  the command (urg3d_library matches `strncmp(data, command, len-1)`),
  while the VER body must start with "vend:" (matched via dataHead).

  Angle tables (VSSP <= 2.2, spotCount <= 256):
      body = "<command echo>\\n" + "&".join("%04X" hex values)
      urg3d_library reads 4 hex chars every 5 bytes starting at
      offset 9 (len("GET:tblh\\n") == len("GET:tv00\\n") == 9).

  _ro/_ri range packet body:
      range_header (22B): u16 header_length, u32 ts_head, u32 ts_tail,
          i16 head_motor_ratio, i16 tail_motor_ratio, u8 frame,
          u8 motor_field, u16 line, u16 spot, u8 rem_field,
          u8 rem_interlace
      range_index: u16 index_length, u16 nspots,
          u16 index[nspots + 1] (cumulative echo offsets),
          u16 reserve padding when nspots is even
      data: per echo u16 range_mm (+ u16 intensity for _ri)

  _ax auxiliary packet body:
      ax_header (12B): u16 header_length, u32 timestamp_ms,
          u32 data_bitfield, u8 data_count, u8 data_ms
      data: data_count records of i32 values, one per bitfield bit
"""

import math
import struct
import threading

import numpy as np

VSSP_PORT = 10940

# YVT-35LX constants — must match urg3d_library's setConstantParameter()
# (the VSSP <= 2.2 fixed sensor specification).
SPOT_COUNT = 74
LINE_COUNT = 36
MAX_MOTOR_INTERLACE = 20
MAX_REM_INTERLACE = 10

# Datasheet measurement range; points outside it get no echo.
MIN_RANGE_M = 0.3
MAX_RANGE_M = 35.0

HEADER_LEN = 24
RANGE_HEADER_LEN = 22
STATUS_OK = '000'
STATUS_ERROR = '0Ee'

# _ax bitfield: gyro XYZ | accel XYZ | compass XYZ | temperature
AX_BITFIELD = 0xFFC00000
AX_VALUES_PER_RECORD = 10

# Raw auxiliary scale factors (urg3d_node2/auxiliary_define.hpp)
GYRO_SCALE = 500.0 / 0x7FFF          # deg/s per LSB
ACCEL_SCALE = 4.0 / 0x7FFF           # G per LSB
COMPASS_SCALE = 0.15                 # uT per LSB
TEMPERATURE_FACTOR = 333.87          # LSB per degC
TEMPERATURE_OFFSET = 21.0            # degC at raw 0


def build_packet(ptype: str, status: str, body: bytes, ts_ms: int) -> bytes:
    """Wrap a body in the 24-byte VSSP common header."""
    ts = int(ts_ms) & 0xFFFFFFFF
    header = struct.pack(
        '<4s4s4sHHII',
        b'VSSP',
        ptype.encode('ascii') + b'\n',
        status.encode('ascii') + b'\n',
        HEADER_LEN,
        HEADER_LEN + len(body),
        ts, ts)
    return header + body


def build_response(command: str, status: str, payload: str,
                   ts_ms: int) -> bytes:
    """Command response: type = first 3 chars, body echoes the command."""
    ptype = (command + '   ')[:3]
    body = (command + '\n' + payload).encode('ascii')
    return build_packet(ptype, status, body, ts_ms)


def build_ver_response(vendor: str, product: str, serial: str,
                       firmware: str, protocol: str, ts_ms: int) -> bytes:
    """VER response. Body must start with "vend:" (no command echo)."""
    body = (f'vend:{vendor}\n'
            f'prod:{product}\n'
            f'seri:{serial}\n'
            f'firm:{firmware}\n'
            f'prot:{protocol}\n').encode('ascii')
    return build_packet('VER', STATUS_OK, body, ts_ms)


def encode_angle_table(command: str, values, ts_ms: int) -> bytes:
    """GET:tblh / GET:tvNN response: echo + '&'-joined 4-hex values."""
    joined = '&'.join(f'{int(v) & 0xFFFF:04X}' for v in values)
    body = (command + '\n' + joined).encode('ascii')
    return build_packet('GET', STATUS_OK, body, ts_ms)


def motor_ratio_table():
    """tblh values: spot position within a line as ratio * 65535."""
    return [round(s / (SPOT_COUNT - 1) * 65535) for s in range(SPOT_COUNT)]


def vertical_angle_table(v_min_deg: float, v_max_deg: float):
    """tvNN values: per-spot vertical angle as (rad / 2pi) * 65535.

    Negative angles wrap to the upper half of the 0..65535 circle;
    urg3d_library subtracts 2pi when the decoded angle exceeds pi.
    """
    values = []
    for s in range(SPOT_COUNT):
        deg = v_min_deg + (v_max_deg - v_min_deg) * s / (SPOT_COUNT - 1)
        rad = math.radians(deg) % (2.0 * math.pi)
        values.append(round(rad / (2.0 * math.pi) * 65535) & 0xFFFF)
    return values


def angle_deg_to_motor_ratio(deg: float) -> int:
    """Signed i16 motor angle ratio: angle / 360 * 65535."""
    return max(-32768, min(32767, round(deg / 360.0 * 65535.0)))


def build_range_packet(*, with_intensity: bool, frame: int, motor_field: int,
                       rem_field: int, rem_interlace: int, line: int,
                       ts_head_ms: int, ts_tail_ms: int,
                       head_ratio: int, tail_ratio: int,
                       ranges_mm, intensities, ts_ms: int) -> bytes:
    """One scan line as a _ri (or _ro) packet.

    ranges_mm / intensities: length SPOT_COUNT sequences; range 0 means
    "no echo for this spot" (the spot gets zero echoes in the index).
    """
    nspots = SPOT_COUNT
    range_header = struct.pack(
        '<HIIhhBBHHBB',
        RANGE_HEADER_LEN,
        int(ts_head_ms) & 0xFFFFFFFF,
        int(ts_tail_ms) & 0xFFFFFFFF,
        head_ratio, tail_ratio,
        frame & 0xFF, motor_field & 0xFF,
        line & 0xFFFF, 0,               # spot offset: full line per packet
        rem_field & 0xFF, rem_interlace & 0xFF)

    index = [0] * (nspots + 1)
    echo_spots = []
    count = 0
    for s in range(nspots):
        index[s] = count
        if ranges_mm[s] > 0:
            echo_spots.append(s)
            count += 1
    index[nspots] = count

    # index_length covers itself, nspots, the index array and the
    # 2-byte reserve that pads nspots+1 (odd) u16 entries to 4-byte
    # alignment when nspots is even.
    pad = 2 if nspots % 2 == 0 else 0
    index_length = 4 + (nspots + 1) * 2 + pad
    index_block = struct.pack('<HH', index_length, nspots)
    index_block += struct.pack(f'<{nspots + 1}H', *index)
    if pad:
        index_block += b'\x00\x00'

    if with_intensity:
        data = b''.join(
            struct.pack('<HH', int(ranges_mm[s]) & 0xFFFF,
                        int(intensities[s]) & 0xFFFF)
            for s in echo_spots)
        ptype = '_ri'
    else:
        data = b''.join(
            struct.pack('<H', int(ranges_mm[s]) & 0xFFFF)
            for s in echo_spots)
        ptype = '_ro'

    return build_packet(ptype, STATUS_OK,
                        range_header + index_block + data, ts_ms)


def build_ax_packet(ts_ms: int, records, data_ms: int) -> bytes:
    """Auxiliary packet: records = list of 10-value int tuples
    (gyro XYZ, accel XYZ, compass XYZ, temperature) in raw LSB."""
    body = struct.pack('<HIIBB', 12, int(ts_ms) & 0xFFFFFFFF, AX_BITFIELD,
                       len(records), data_ms & 0xFF)
    for rec in records:
        body += struct.pack(f'<{AX_VALUES_PER_RECORD}i',
                            *[int(v) for v in rec])
    return build_packet('_ax', STATUS_OK, body, ts_ms)


class ScanGeometry:
    """Maps between cartesian points and the YVT scan grid.

    The emulated scan mirrors what urg3d_library reconstructs from our
    own tables: within a line the motor (horizontal) angle interpolates
    head->tail with the tblh ratio while the spot's vertical angle
    follows the tvNN table, i.e. a diagonal sweep per line.
    """

    def __init__(self, h_fov_deg: float = 210.0,
                 v_min_deg: float = -20.0, v_max_deg: float = 20.0):
        self.h_min_deg = -h_fov_deg / 2.0
        self.h_max_deg = h_fov_deg / 2.0
        self.v_min_deg = v_min_deg
        self.v_max_deg = v_max_deg
        self.line_width_deg = h_fov_deg / LINE_COUNT
        # Per-line head/tail motor angle ratios (i16)
        self.head_ratios = [
            angle_deg_to_motor_ratio(self.h_min_deg
                                     + li * self.line_width_deg)
            for li in range(LINE_COUNT)]
        self.tail_ratios = [
            angle_deg_to_motor_ratio(self.h_min_deg
                                     + (li + 1) * self.line_width_deg)
            for li in range(LINE_COUNT)]

    def bin_pointcloud(self, x, y, z, intensity):
        """Bin cartesian points into (LINE_COUNT, SPOT_COUNT) grids.

        Returns (range_mm, intensity) uint16 grids; range 0 = no echo.
        Keeps the nearest return per cell.
        """
        range_grid = np.zeros((LINE_COUNT, SPOT_COUNT), dtype=np.uint16)
        intens_grid = np.zeros((LINE_COUNT, SPOT_COUNT), dtype=np.uint16)
        if len(x) == 0:
            return range_grid, intens_grid

        az = np.degrees(np.arctan2(y, x))
        horiz = np.hypot(x, y)
        el = np.degrees(np.arctan2(z, horiz))
        dist = np.sqrt(x * x + y * y + z * z)

        # Points the real sensor could not report (outside its range or
        # its field of view) must not appear in the packets either.
        valid = ((dist >= MIN_RANGE_M) & (dist <= MAX_RANGE_M)
                 & (az >= self.h_min_deg) & (az <= self.h_max_deg)
                 & (el >= self.v_min_deg) & (el <= self.v_max_deg))
        if not np.any(valid):
            return range_grid, intens_grid
        az, el, dist = az[valid], el[valid], dist[valid]
        inten = np.clip(intensity[valid], 0, 65535).astype(np.uint16)

        spot_f = ((el - self.v_min_deg)
                  / (self.v_max_deg - self.v_min_deg) * (SPOT_COUNT - 1))
        spot = np.clip(np.rint(spot_f), 0, SPOT_COUNT - 1).astype(np.intp)
        # Within line li the horizontal angle of spot s is
        # h_min + (li + s/(SPOT_COUNT-1)) * line_width, so invert that
        # to find the line whose sweep passes closest to the point.
        line_f = ((az - self.h_min_deg) / self.line_width_deg
                  - spot / (SPOT_COUNT - 1))
        line = np.clip(np.rint(line_f), 0, LINE_COUNT - 1).astype(np.intp)

        range_mm = np.clip(dist * 1000.0, 1, 65535).astype(np.uint16)

        cell = line * SPOT_COUNT + spot
        order = np.lexsort((range_mm, cell))
        cell_sorted = cell[order]
        first = np.ones(len(order), dtype=bool)
        first[1:] = cell_sorted[1:] != cell_sorted[:-1]
        sel = order[first]

        range_grid.reshape(-1)[cell[sel]] = range_mm[sel]
        intens_grid.reshape(-1)[cell[sel]] = inten[sel]
        return range_grid, intens_grid


class RangeNoise:
    """Per-spot range noise and dropout for a binned scan grid.

    A real sensor's error lands on the measured distance along the beam,
    so it is applied here - to the uint16 millimetre values that go into
    the packet - rather than to the cartesian points coming in. Noise is
    off unless configured, because the simulation feeding the emulator
    usually models sensor noise already and applying it twice would
    understate the driver's real input quality.

    Applied once per revolution rather than once per incoming cloud, so
    a static scene still produces an independent measurement per scan,
    as a real sensor does.

    Reproducible: with the same seed and the same call sequence, the
    same measurements come out.
    """

    def __init__(self, sigma_m: float = 0.0,
                 dropout_probability: float = 0.0, seed: int = 0):
        self.sigma_mm = max(0.0, float(sigma_m)) * 1000.0
        self.dropout_probability = min(1.0, max(0.0, float(dropout_probability)))
        self._rng = np.random.default_rng(seed)
        # apply() is called once per revolution, from each connection's
        # streaming thread; a Generator is not thread-safe.
        self._lock = threading.Lock()

    @property
    def enabled(self) -> bool:
        return self.sigma_mm > 0.0 or self.dropout_probability > 0.0

    def apply(self, range_grid, intensity_grid):
        """Return (range, intensity) grids with noise and dropout applied."""
        if not self.enabled:
            return range_grid, intensity_grid

        noisy = range_grid.astype(np.int32)
        flat = noisy.reshape(-1)
        hit = np.flatnonzero(flat > 0)
        if hit.size == 0:
            return range_grid, intensity_grid

        with self._lock:
            if self.sigma_mm > 0.0:
                flat[hit] += np.rint(self._rng.normal(
                    0.0, self.sigma_mm, hit.size)).astype(np.int32)
            if self.dropout_probability > 0.0:
                dropped = hit[
                    self._rng.random(hit.size) < self.dropout_probability]
                flat[dropped] = 0

        # A return pushed outside the sensor's measurement range is not
        # reported at all, as on the real device.
        lo = int(MIN_RANGE_M * 1000.0)
        hi = int(MAX_RANGE_M * 1000.0)
        flat[(flat < lo) | (flat > hi)] = 0

        out_range = noisy.astype(np.uint16)
        out_intensity = np.where(out_range > 0, intensity_grid,
                                 0).astype(np.uint16)
        return out_range, out_intensity


def aux_record_raw(gyro_dps=(0.0, 0.0, 0.0), accel_g=(0.0, 0.0, 1.0),
                   compass_ut=(30.0, 0.0, -40.0), temp_c=25.0):
    """Convert physical auxiliary values to raw LSB tuple for _ax."""
    return (
        round(gyro_dps[0] / GYRO_SCALE),
        round(gyro_dps[1] / GYRO_SCALE),
        round(gyro_dps[2] / GYRO_SCALE),
        round(accel_g[0] / ACCEL_SCALE),
        round(accel_g[1] / ACCEL_SCALE),
        round(accel_g[2] / ACCEL_SCALE),
        round(compass_ut[0] / COMPASS_SCALE),
        round(compass_ut[1] / COMPASS_SCALE),
        round(compass_ut[2] / COMPASS_SCALE),
        round((temp_c - TEMPERATURE_OFFSET) * TEMPERATURE_FACTOR),
    )
