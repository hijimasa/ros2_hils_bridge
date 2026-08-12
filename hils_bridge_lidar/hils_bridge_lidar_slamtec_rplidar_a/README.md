# hils_bridge_lidar_slamtec_rplidar_a

Emulator bridge for **SLAMTEC RPLIDAR** 2D LiDARs speaking the
A-series **serial** protocol (0xA5 request / response-descriptor /
5-byte scan-node stream). The real host-side driver (`rplidar_ros` /
the SLAMTEC SDK) opens the serial port and issues requests exactly as
it would to the real sensor; this node answers them and streams a
subscribed `sensor_msgs/LaserScan` topic as measurement nodes.

Python port of REACT-simulator's `rplidar_emulator` (C++), rebuilt on
`SerialBridgeBase` so the standard HILS fault pipeline (drop / corrupt
/ delay / freeze / duplicate ...) and device state machine apply to
every reply and to the scan stream.

```
rplidar driver (host) ──0xA5 requests──▶ FT234X or PTY ──▶ rplidar_bridge_node ◀──LaserScan── simulator
        ◀──descriptors + replies ('command') / 5-byte node stream ('scan')──
```

## Protocol coverage

| Request | Code | Reply |
|---------|------|-------|
| GET_DEVICE_INFO | `0x50` | descriptor + 20 bytes (model, fw, hw, serial) |
| GET_DEVICE_HEALTH | `0x52` | descriptor + 3 bytes (always `STATUS_OK`) |
| GET_SAMPLERATE | `0x59` | descriptor + 4 bytes (Tstandard/Texpress, 130 µs) |
| GET_LIDAR_CONF | `0x84` | descriptor + echoed type + data (single "Standard" mode) |
| SCAN / FORCE_SCAN | `0x20` / `0x21` | continuous descriptor, then the node stream |
| EXPRESS_SCAN / HQ_SCAN | `0x82` / `0x83` | matching descriptor, stream falls back to standard nodes |
| STOP | `0x25` | no reply; stream stops |
| RESET | `0x40` | no reply; stream stops |
| SET_MOTOR_PWM / HQ_MOTOR_SPEED | `0xF0` / `0xA8` | accepted, ignored |

Scan nodes are the standard 5-byte format: `sync_quality` (bit 0
syncbit, set on the first node of each revolution; bit 1 inverted
syncbit; bits 2-7 quality), `angle_q6_checkbit` (bit 0 checkbit,
bits 1-15 angle in deg × 64, wire CW so the ROS CCW angle is negated),
`distance_q2` (mm × 4, 0 = invalid, clamped at 65534). One revolution
is streamed every 100 ms (10 Hz) while scanning. When the scan topic
is silent (or all points invalid), a synthetic 360-point pattern is
streamed instead so the driver still sees data — same behavior as the
C++ emulator.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scan_topic` | string | `/scan` | Subscribed `LaserScan` from the simulator |
| `serial_port` | string | `/tmp/rplidar_front` | Serial device, or the symlink path to create when `create_pty` |
| `create_pty` | bool | `false` (`true` in `default_params.yaml`) | Create a virtual serial device (PTY) at `serial_port` (SILS, no adapter) |
| `baudrate` | int | `1000000` | Match the driver (meaningless on a PTY) |
| `model_code` | int | `0x61` (S2) | GET_DEVICE_INFO model byte |
| `firmware_version` | int | `0x0118` (v1.24) | GET_DEVICE_INFO firmware (16 bit) |
| `hardware_version` | int | `0x07` | GET_DEVICE_INFO hardware byte |
| `serial_number` | string | `EMULATOR00000001` | GET_DEVICE_INFO serial (16 bytes, padded/truncated) |

## Usage

SILS (virtual serial device, no adapter):

```bash
ros2 launch hils_bridge_lidar_slamtec_rplidar_a rplidar_bridge.launch.py

# or directly:
ros2 run hils_bridge_lidar_slamtec_rplidar_a rplidar_bridge_node --ros-args \
    -p create_pty:=true -p serial_port:=/tmp/rplidar_front \
    -p scan_topic:=/scan

# the driver then opens the symlink as if it were the real sensor:
ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/tmp/rplidar_front
```

HILS (FT234X cross-connection): `create_pty:=false
serial_port:=/dev/ttyUSB0` on this side, the driver on the robot PC's
own port.

Fault injection uses the standard services on `/hils_rplidar_bridge`
(`~/inject_fault`, `~/clear_fault`, `~/get_fault_state`,
`~/set_device_state`). Command replies go out on the `command`
channel, the measurement stream on the `scan` channel, so e.g.

```bash
ros2 service call /hils_rplidar_bridge/inject_fault \
  hils_bridge_interfaces/srv/InjectFault \
  "{fault_yaml: 'fault_type: drop\ntarget: scan\nparameters: {probability: 1.0}'}"
```

mutes the scan stream while GET_HEALTH etc. still answer — the
"motor stalled but the MCU is alive" failure mode.

## Notes / limitations

- Express (`0x82`) / HQ (`0x83`) scans are acknowledged with the
  matching descriptor but the stream falls back to standard 5-byte
  nodes; only the one "Standard" scan mode is advertised (like the
  C++ emulator).
- `GET_SAMPLERATE` (`0x59`) is answered (A-series hosts require it);
  the C++ emulator did not implement it.
- Request checksums are not verified (the C++ emulator does not
  either); resync is by scanning for the `0xA5` sync byte.
- GET_DEVICE_HEALTH always reports OK; health-code error injection is
  not ported (use the generic fault pipeline / device states instead).
- Timing is idealized: one revolution per 100 ms burst rather than a
  continuous per-sample cadence.

## Tests

`python3 -m pytest test -q` (20 tests, no ROS required) pins the
request parse/resync, response-descriptor and reply byte layouts,
request checksum, sync/checkbit flags, and the angle/distance
encodings against the C++ formulas.
