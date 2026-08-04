#!/usr/bin/env python3
"""Pin down which corrupted VSSP field crashes urg3d_node2.

Serves a minimal, valid VSSP 2.1 session (no ROS, no emulator), streams
healthy _ri lines for a few seconds, then injects ONE packet whose
range-header field under test carries an out-of-range value. Everything
else in that packet stays well-formed, so a crash isolates the field.

    python3 probe_urg3d_range_header.py --field spot --value 8000

Fields: spot, nspots, header_length, rem_field, line, none (control).
Run the driver against 127.0.0.1:10940 while this is listening; the
script reports whether the client disconnected after the bad packet.
"""

import argparse
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'hils_bridge_lidar',
    'hils_bridge_lidar_hokuyo_yvt35lx'))

from hils_bridge_lidar_hokuyo_yvt35lx import vssp_protocol as vp  # noqa: E402


def bad_range_packet(field, value, ts_ms, line):
    """A structurally valid _ri packet with one header field overridden."""
    header_length = value if field == 'header_length' else vp.RANGE_HEADER_LEN
    spot = value if field == 'spot' else 0
    nspots = value if field == 'nspots' else vp.SPOT_COUNT
    rem_field = value if field == 'rem_field' else 0
    line_no = value if field == 'line' else line

    range_header = struct.pack(
        '<HIIhhBBHHBB', header_length & 0xFFFF, ts_ms, ts_ms + 3,
        0, 1820, 0, 0, line_no & 0xFFFF, spot & 0xFFFF, rem_field & 0xFF, 1)

    # Index block describes SPOT_COUNT real spots with one echo each,
    # but advertises the (possibly bogus) nspots the driver will trust.
    echo_count = vp.SPOT_COUNT
    index = list(range(vp.SPOT_COUNT + 1))
    pad = 2 if (nspots & 0xFFFF) % 2 == 0 else 0
    index_length = 4 + (vp.SPOT_COUNT + 1) * 2 + pad
    body = range_header
    body += struct.pack('<HH', index_length, nspots & 0xFFFF)
    body += struct.pack(f'<{vp.SPOT_COUNT + 1}H', *index)
    if pad:
        body += b'\x00\x00'
    body += b''.join(struct.pack('<HH', 3000, 100) for _ in range(echo_count))
    return vp.build_packet('_ri', vp.STATUS_OK, body, ts_ms)


def good_range_packet(ts_ms, line):
    ranges = [3000] * vp.SPOT_COUNT
    intens = [100] * vp.SPOT_COUNT
    geom = vp.ScanGeometry()
    return vp.build_range_packet(
        with_intensity=True, frame=0, motor_field=0, rem_field=0,
        rem_interlace=1, line=line, ts_head_ms=ts_ms, ts_tail_ms=ts_ms + 3,
        head_ratio=geom.head_ratios[line], tail_ratio=geom.tail_ratios[line],
        ranges_mm=ranges, intensities=intens, ts_ms=ts_ms)


def serve(conn, field, value, healthy_sec):
    epoch = time.monotonic()
    rx = b''
    streaming = False
    conn.settimeout(0.01)
    injected = False
    line = 0
    packets_after = 0

    while True:
        ts = int((time.monotonic() - epoch) * 1000)
        try:
            data = conn.recv(4096)
            if not data:
                return 'client closed', packets_after
            rx += data
        except (socket.timeout, BlockingIOError):
            pass
        except OSError as e:
            return f'recv error: {e}', packets_after

        while b'\n' in rx:
            cmd, rx = rx.split(b'\n', 1)
            command = cmd.decode('ascii', 'replace')
            if command == 'VER':
                pkt = vp.build_ver_response(
                    'HOKUYO AUTOMATIC CO.,LTD.', 'YVT-35LX', 'PROBE',
                    'probe', 'VSSP2.1', ts)
            elif command == 'GET:stat':
                pkt = vp.build_response(command, vp.STATUS_OK,
                                        '_ro=000\n_ri=000\n_ax=000\n', ts)
            elif command in ('GET:_itl', 'GET:_itv'):
                pkt = vp.build_response(command, vp.STATUS_OK, '0,01\n', ts)
            elif command == 'GET:tblh':
                pkt = vp.encode_angle_table(command, vp.motor_ratio_table(), ts)
            elif command.startswith('GET:tv'):
                pkt = vp.encode_angle_table(
                    command, vp.vertical_angle_table(-20.0, 20.0), ts)
            elif command.startswith('DAT:ri='):
                streaming = command.endswith('1')
                pkt = vp.build_response(command, vp.STATUS_OK, '', ts)
            else:
                pkt = vp.build_response(command, vp.STATUS_OK, '', ts)
            try:
                conn.sendall(pkt)
            except OSError as e:
                return f'send error on {command}: {e}', packets_after

        if streaming:
            elapsed = time.monotonic() - epoch
            try:
                if not injected and elapsed > healthy_sec and field != 'none':
                    conn.sendall(bad_range_packet(field, value, ts, line))
                    injected = True
                    print(f'[probe] injected bad {field}={value} at '
                          f't={elapsed:.1f}s', flush=True)
                else:
                    conn.sendall(good_range_packet(ts, line))
                    if injected:
                        packets_after += 1
            except OSError as e:
                return f'send error while streaming: {e}', packets_after
            line = (line + 1) % vp.LINE_COUNT
            time.sleep(1.0 / 360.0)
            if injected and packets_after > 2000:
                return 'survived', packets_after
        else:
            time.sleep(0.005)
        if time.monotonic() - epoch > healthy_sec + 30:
            return 'timeout', packets_after


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--field', default='spot',
                    choices=['spot', 'nspots', 'header_length', 'rem_field',
                             'line', 'none'])
    ap.add_argument('--value', type=int, default=8000)
    ap.add_argument('--healthy-sec', type=float, default=5.0)
    ap.add_argument('--port', type=int, default=10940)
    args = ap.parse_args()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(('127.0.0.1', args.port))
    srv.listen(1)
    print(f'[probe] listening on 127.0.0.1:{args.port}, '
          f'field={args.field} value={args.value}', flush=True)
    conn, addr = srv.accept()
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(f'[probe] driver connected from {addr}', flush=True)
    verdict, after = serve(conn, args.field, args.value, args.healthy_sec)
    print(f'[probe] RESULT field={args.field} value={args.value} '
          f'verdict={verdict} packets_after_injection={after}', flush=True)
    conn.close()
    srv.close()


if __name__ == '__main__':
    main()
