# hils_bridge_actuator_orientalmotor_id_share

Emulator bridge for OrientalMotor **BLV-R** / **AZ** series motors on
an **ID-share** Modbus RTU bus (one serial line, multiple motors).
The real robot-side motor daemon (`orientalmotor_id_share_daemon`, the
bus master) polls this node exactly as it would poll the real drivers;
the node converts drive commands to `JointState` topics for the
simulator and simulator feedback back into driver-native replies.

Python port of REACT-simulator's `orientalmotor_id_share_emulator`
(C++), rebuilt on `SerialBridgeBase` so the standard HILS fault
pipeline (drop / corrupt / delay / freeze / duplicate ...) and device
state machine apply to every reply.

```
motor daemon (master) ──Modbus RTU──▶ FT234X or PTY ──▶ oms_bridge_node ──JointState──▶ simulator
     (robot PC)                        (serial path)      (slaves, this)    feedback  ◀──
```

The daemon only ever issues two function codes, both answered here:

| Function | Meaning | Reply |
|----------|---------|-------|
| `0x10` Write Multiple Registers | ID-share / read / write register setup, zero-speed | 8-byte echo |
| `0x17` Read/Write Multiple Registers ("Direct Data Drive") | per-cycle command, 32 bytes/motor | feedback, 30 bytes/motor |

## Two motor series, one protocol

The ID-share register map is common to BLV-R and AZ; the wire carries
**driver-native integers** whose units differ per series. This node
converts motor-shaft native units ↔ output-shaft joint rad / rad/s / Nm
per motor:

| Item | BLV-R | AZ |
|------|-------|----|
| Native velocity unit | rpm | Hz (= rpm/60 × step/rev) |
| Steps per revolution | 36000 | 1000 |
| Torque | 0.1% of rated (1000 = 100%) | same (base = holding torque) |

Rated torque is looked up from the `motor_types` KEY: BLV-R output
power [W] (60/100/200/400), AZ frame size [mm] (20/28/42/60/85) — the
same `TYPE[:KEY]` syntax as the daemon's `--motor-types`.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motor_types` | string | `""` (**required**) | Comma-separated `TYPE[:KEY]` (e.g. `BLVR:100,AZ:60,BLVR:200`). The list length sets the motor count |
| `serial_port` | string | `/tmp/ttyUSB_BlvrMotor` | Serial device, or the symlink path to create when `create_pty` |
| `create_pty` | bool | `false` (`true` in `default_params.yaml`) | Create a virtual serial device (PTY) at `serial_port` (SILS, no adapter) |
| `baudrate` | int | `230400` | Match the daemon's `BAUDRATE` (meaningless on a PTY) |
| `joint_names` | string[] | `[motor_0, ...]` | Motor id → joint name mapping |
| `gear_ratios` | double[] | `[1.0]` | Motor:output reduction. Empty = all 1.0 / one entry = uniform / N entries = per motor (zero is a startup error) |
| `joint_states_topic` | string | `/joint_states` | Subscribed (output-shaft feedback) |
| `joint_commands_topic` | string | `/joint_commands` | Published (output-shaft commands) |

## Control modes and JointState

Motors on one bus may be in different control modes (e.g. wheels by
velocity, steering by position). Like
`topic_based_ros2_control::TopicBasedSystem`, up to **three separate
`JointState` messages** (position / velocity / effort) are published
per poll, each containing **only the joints commanded on that
interface** with `name[]` aligned to the single populated array — a
joint without a command is omitted, never filled with `0.0`/`NaN`.

| Mode | Value | Published as |
|------|-------|--------------|
| ABSOLUTE_POSITION / RELATIVE_* / RETURN_HOME | 1/2/3/23 | position [rad] (RETURN_HOME → 0.0) |
| CONTINUOUS_OPERATION_BY_RPM(_CYCLIC) | 48 / 51 | velocity [rad/s] |
| CONTINUOUS_OPERATION_BY_TORQUE | 50 | effort [Nm] (motor Nm × gear) |
| NOCONTROL / unknown | 0 | omitted from every message |

Feedback: `joint_states` position/velocity are matched by joint name,
converted through the gear ratio and the series' step/rev / velocity
unit, and returned in the next `0x17` reply.

## Usage

SILS (virtual serial device, no adapter):

```bash
ros2 launch hils_bridge_actuator_orientalmotor_id_share oms_bridge.launch.py

# or directly:
ros2 run hils_bridge_actuator_orientalmotor_id_share oms_bridge_node --ros-args \
    -p motor_types:="BLVR:400,BLVR:400" \
    -p create_pty:=true -p serial_port:=/tmp/ttyUSB_BlvrMotor \
    -p joint_names:="[left_wheel_joint,right_wheel_joint]" \
    -p gear_ratios:="[50.0]"

# the daemon then opens the symlink as if it were the real bus:
orientalmotor_id_share_daemon -t BLVR:400,BLVR:400 -d /tmp/ttyUSB_BlvrMotor -f 50
```

HILS (FT234X cross-connection): `create_pty:=false
serial_port:=/dev/ttyUSB0` on this side, the daemon on the robot PC's
own port.

Fault injection uses the standard services on `/hils_oms_bridge`
(`~/inject_fault`, `~/clear_fault`, `~/get_fault_state`,
`~/set_device_state`); replies go out on the `status` channel. See
`hils_bringup/scenarios/actuator/oms_reply_blackout_001.yaml`.

## Notes / limitations

- Alarm / communication-error injection beyond the generic byte-level
  faults is not implemented (`motor_alarm` / `communication_error`
  are always 0).
- Return-to-home (mode 0 + dedicated driver input on the real daemon)
  is not emulated; position / velocity / torque control are.
- `stop_flag` (driver output bit 5) is always reported cleared: the
  controller treats it as a hardware STOP input, so asserting it when
  idle would deadlock startup.

## Tests

`python3 -m pytest test -q` (12 tests, no ROS required) pins the
Modbus frame layout, CRC, resync behavior, and the BLV-R / AZ unit
conversions.
