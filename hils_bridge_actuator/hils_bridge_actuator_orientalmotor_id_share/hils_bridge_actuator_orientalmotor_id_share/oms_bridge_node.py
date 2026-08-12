#!/usr/bin/env python3
"""
HILS OrientalMotor ID-share Motor Bridge Node

Emulates OrientalMotor BLV-R / AZ series motors on an ID-share Modbus
RTU bus (one serial line, multiple motors) so the real robot-side
motor daemon (orientalmotor_id_share_daemon, the bus master) can be
exercised without hardware. The daemon polls; this node answers.

Python port of REACT-simulator's orientalmotor_id_share_emulator
(oms_emulator_node.cpp), rebuilt on SerialBridgeBase so the standard
HILS fault pipeline (drop / corrupt / delay / freeze / duplicate ...)
and device state machine apply to every reply.

Data flow:
    Motor daemon (master) --Modbus RTU--> FT234X or PTY --> this node
        0x10 setup writes            -> echoed back
        0x17 Direct Data Drive polls -> JointState on joint_commands_topic
                                        (position/velocity/effort, out-shaft)
    Simulator --JointState on joint_states_topic--> this node
        position/velocity feedback   -> stored, returned in the next
                                        0x17 reply in driver-native units

The motor mix is configured with a `motor_types` list ("BLVR:100,AZ:60"),
the same syntax as the daemon's --motor-types: each motor's series sets
its step/rev (36000 vs 1000), native velocity unit (rpm vs Hz) and
rated torque, applied per motor when converting between the bus's
driver-native integers and output-shaft rad / rad/s / Nm.
"""

import threading
import time

import rclpy
import serial
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState

from hils_bridge_base.serial_bridge_base import SerialBridgeBase

from hils_bridge_actuator_orientalmotor_id_share import oms_motor_spec as spec
from hils_bridge_actuator_orientalmotor_id_share import oms_protocol as proto


class OmsBridgeNode(SerialBridgeBase):
    """ROS2 node that answers ID-share Modbus polls as the motor slaves."""

    def __init__(self):
        super().__init__(
            node_name='hils_oms_bridge',
            default_baudrate=230400,
            default_serial_port='/tmp/ttyUSB_BlvrMotor',
            frame_protocol_firmware=False,  # raw vendor wire protocol
        )

        # Motor configuration (same syntax as the daemon's --motor-types;
        # the list length sets the motor count).
        self.declare_parameter('motor_types', '',
            ParameterDescriptor(
                description='Comma-separated TYPE[:KEY] list, e.g. '
                            '"BLVR:100,AZ:60,BLVR:200". KEY is BLV-R '
                            'output power [W] or AZ frame size [mm]. '
                            'Required.'))
        self.declare_parameter('joint_names', [''],
            ParameterDescriptor(
                description='Ordered joint name per motor. Empty string '
                            'means "motor_<index>".'))
        self.declare_parameter('gear_ratios', [1.0],
            ParameterDescriptor(
                description='Motor:output reduction. Empty = all 1.0, '
                            'one entry = uniform, N entries = per motor.'))
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_commands_topic', '/joint_commands')

        # Resolve the motor list. A bad list is a configuration error:
        # fail startup loudly (like the base does on a bad serial port).
        motor_types = self.get_parameter('motor_types').value
        try:
            self._motor_specs = spec.parse_motor_types(motor_types)
        except ValueError as e:
            self.get_logger().error(
                f'Invalid \'motor_types\' ("{motor_types}"): {e}')
            self.get_logger().error(
                'Example: -p motor_types:="BLVR:100,AZ:60,BLVR:200"')
            raise
        self._motor_count = len(self._motor_specs)

        # Fill in default joint names for any motors the user did not name.
        names = [n for n in self.get_parameter('joint_names').value if n]
        while len(names) < self._motor_count:
            names.append(f'motor_{len(names)}')
        self._joint_names = names[:self._motor_count]

        # Resolve per-motor gear ratios (empty -> 1.0, single -> uniform,
        # N -> per motor); same rules as the daemon-side emulator.
        self._gear_ratios = self._resolve_gear_ratios(
            list(self.get_parameter('gear_ratios').value))

        # Motor feedback state, in driver-native integer units. Written
        # by the joint_states subscription and the 0x17 command path,
        # read when building the 0x17 reply.
        self._motor_states = [proto.MotorResponse()
                              for _ in range(self._motor_count)]
        self._motor_states_lock = threading.Lock()

        # ROS interfaces (simulator-facing)
        joint_states_topic = self.get_parameter('joint_states_topic').value
        joint_commands_topic = self.get_parameter('joint_commands_topic').value
        self.create_subscription(
            JointState, joint_states_topic, self._joint_states_callback, 10)
        self._joint_commands_pub = self.create_publisher(
            JointState, joint_commands_topic, 10)

        # Serial side (daemon-facing): background RX thread. The bus is
        # master-driven, so everything this node sends is a reply.
        self._rx_buffer = bytearray()
        self._stop_event = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._serial_read_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            f'OrientalMotor ID-share bridge started: motors={self._motor_count}, '
            f'joint_states={joint_states_topic}, '
            f'joint_commands={joint_commands_topic}')
        for i, m in enumerate(self._motor_specs):
            self.get_logger().info(
                f'  motor {i}: {spec.motor_type_name(m.type)} (key={m.key}, '
                f'rated={m.rated_torque_nm:.3f} Nm, {m.step_per_rev} step/rev), '
                f'gear={self._gear_ratios[i]:.3f}:1 '
                f'-> joint "{self._joint_names[i]}"')

    def _resolve_gear_ratios(self, raw):
        """Resolve the gear_ratios parameter into one value per motor.

        Raises:
            ValueError: on a bad list size or a zero ratio.
        """
        if not raw:
            ratios = [1.0] * self._motor_count
        elif len(raw) == 1:
            ratios = [raw[0]] * self._motor_count
        elif len(raw) == self._motor_count:
            ratios = raw
        else:
            msg = (f"'gear_ratios' has {len(raw)} entries; expected 0 "
                   f'(all 1.0), 1 (uniform), or {self._motor_count} '
                   f'(per motor)')
            self.get_logger().error(msg)
            raise ValueError(msg)

        for i, ratio in enumerate(ratios):
            if ratio == 0.0:
                msg = f"'gear_ratios[{i}]' must be non-zero"
                self.get_logger().error(msg)
                raise ValueError(msg)
        return ratios

    # ---------- Serial input (daemon side) ----------

    def _serial_read_loop(self):
        """Background thread: drain the serial port and answer frames."""
        while not self._stop_event.is_set() and rclpy.ok():
            try:
                data = self._serial.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read failed: {e}')
                return

            if not data:
                time.sleep(0.005)
                continue

            self._rx_buffer.extend(data)
            while True:
                frame = proto.extract_frame(self._rx_buffer)
                if frame is None:
                    break
                self._process_frame(frame)

    def _process_frame(self, frame: bytes):
        """Handle one CRC-valid Modbus frame from the master."""
        # Device state gate on the REQUEST side: a powered-off /
        # rebooting device does not parse requests. The frame is
        # consumed but no reply goes out -- the bus goes silent, which
        # is exactly what the daemon sees on a dead motor.
        if not self.device_state.is_open('command'):
            return

        # 0x10: setup/configuration writes (ID-share, read/write
        # register, zero-speed). Reply = echo of the first 6 bytes.
        if proto.is_setup_frame(frame):
            response = proto.build_setup_echo(frame)
            if response is not None:
                # channel='status': replies ride the fault pipeline and
                # device state gate. No check_rate_limit(): the master
                # sets the poll rate, a slave must answer every poll.
                self.serial_write(response, channel='status')
            return

        # 0x17: per-cycle Direct Data Drive command.
        if proto.is_direct_data_drive_frame(frame):
            parsed = proto.parse_direct_data_drive(frame)
            if parsed is None:
                self.get_logger().warning(
                    f'Failed to parse Direct Data Drive frame '
                    f'(size={len(frame)})', throttle_duration_sec=1.0)
                return
            global_id, requests = parsed

            self._publish_joint_commands(requests)
            self._apply_command_to_state(requests)

            with self._motor_states_lock:
                response = proto.build_direct_data_drive_response(
                    global_id, self._motor_states)
            self.serial_write(response, channel='status')
            return

        self.get_logger().warning(
            f'Unhandled frame (func=0x{frame[1]:02X})'
            if len(frame) >= 2 else 'Unhandled short frame',
            throttle_duration_sec=1.0)

    # ---------- Publishing (simulator side) ----------

    def _publish_joint_commands(self, requests):
        """Publish the master's command as JointState message(s).

        Each motor on the bus may be in a different control mode (e.g.
        a robot drives its wheels by velocity while steering by
        position). We mirror exactly what
        topic_based_ros2_control::TopicBasedSystem publishes (the
        format the sim is built to consume): up to THREE separate
        JointState messages, one per interface, each containing ONLY
        the joints commanded on that interface with name[] aligned to
        the single populated array. "No command" for a joint on a given
        interface is therefore expressed by OMISSION (it is absent from
        that message) rather than a sentinel value -- no ambiguous
        0.0 / NaN fillers.
        """
        if not requests:
            return

        n = min(len(requests), self._motor_count)
        stamp = self.get_clock().now().to_msg()

        position_msg = JointState()
        velocity_msg = JointState()
        effort_msg = JointState()
        position_msg.header.stamp = stamp
        velocity_msg.header.stamp = stamp
        effort_msg.header.stamp = stamp

        for i in range(n):
            req = requests[i]
            motor = self._motor_specs[i]
            gear = self._gear_ratios[i]

            if proto.is_position_mode(req.mode):
                position_msg.name.append(self._joint_names[i])
                position_msg.position.append(
                    0.0 if req.mode == proto.MODE_RETURN_HOME_POSITION
                    else spec.motor_step_to_output_rad(
                        req.target_position, motor, gear))
            elif proto.is_velocity_mode(req.mode):
                velocity_msg.name.append(self._joint_names[i])
                velocity_msg.velocity.append(
                    spec.native_vel_to_output_rad_s(
                        req.target_velocity, motor, gear))
            elif proto.is_effort_mode(req.mode):
                # Motor-shaft Nm reflected through the gearbox to the
                # output joint.
                motor_nm = spec.permille_to_nm(
                    req.limit_torque, motor.rated_torque_nm)
                effort_msg.name.append(self._joint_names[i])
                effort_msg.effort.append(motor_nm * gear)
            # MODE_NOCONTROL / unknown: this joint is omitted from
            # every message.

        if position_msg.name:
            self._joint_commands_pub.publish(position_msg)
        if velocity_msg.name:
            self._joint_commands_pub.publish(velocity_msg)
        if effort_msg.name:
            self._joint_commands_pub.publish(effort_msg)

    def _apply_command_to_state(self, requests):
        """Carry command-echo quantities into the feedback state.

        Position and velocity feedback come from the simulator
        (_joint_states_callback); here we only carry through quantities
        the simulator does not report back: the commanded torque limit
        and the free flag.
        """
        with self._motor_states_lock:
            n = min(len(requests), self._motor_count)
            for i in range(n):
                self._motor_states[i].current_torque = requests[i].limit_torque
                self._motor_states[i].free_flag = requests[i].free
                # NOTE: Do NOT assert stop_flag from "velocity == 0".
                # The controller treats stop_flag (BLV-R driver output
                # bit 5) as a hardware STOP input that halts driving,
                # so setting it whenever the motor is idle deadlocks
                # startup (stopped -> stop_flag=1 -> controller
                # commands 0 -> stays stopped). Real BLV-R leaves it
                # cleared unless a STOP input is active, which this
                # bridge does not model.
                self._motor_states[i].stop_flag = False

    # ---------- Feedback (simulator side) ----------

    def _joint_states_callback(self, msg: JointState):
        """Store simulator feedback for the next 0x17 reply.

        Maps output-shaft joint feedback to motor-native state, by
        joint NAME (the simulator may publish joints in any order),
        applying the gear ratio.
        """
        with self._motor_states_lock:
            for motor_id in range(self._motor_count):
                wanted = self._joint_names[motor_id]
                try:
                    idx = list(msg.name).index(wanted)
                except ValueError:
                    continue
                motor = self._motor_specs[motor_id]
                gear = self._gear_ratios[motor_id]

                if idx < len(msg.position):
                    self._motor_states[motor_id].current_position = \
                        spec.output_rad_to_motor_step(
                            msg.position[idx], motor, gear)
                if idx < len(msg.velocity):
                    self._motor_states[motor_id].current_velocity = \
                        spec.output_rad_s_to_native_vel(
                            msg.velocity[idx], motor, gear)

    # ---------- Housekeeping ----------

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, '_rx_thread') and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmsBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
