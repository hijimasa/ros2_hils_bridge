# hils_bridge_actuator

Actuator-path HILS bridges. Each sub-package sits on the wire between the real robot controller and its actuators: it either captures the signal the controller emits and republishes the measurement to ROS, or fully emulates the actuator's slave side of a bidirectional bus so the controller's real driver stack can be exercised.

| Package | Captured / Emulated Signal | Firmware |
|---------|----------------------------|----------|
| [hils_bridge_actuator_servo_pwm](hils_bridge_actuator_servo_pwm/) | RC servo PWM (50 Hz, 500–2500 us pulses), capture | rp2040_actuator_servo_pwm |
| [hils_bridge_actuator_orientalmotor_id_share](hils_bridge_actuator_orientalmotor_id_share/) | OrientalMotor BLV-R / AZ ID-share Modbus RTU bus, slave emulation | — (FT234X or PTY, no MCU) |

PWM with 50 Hz / 1–2 ms pulses is the de-facto standard for RC servo control across virtually all vendors, so `hils_bridge_actuator_servo_pwm` is named for the protocol rather than a specific vendor. ID-share is OrientalMotor's vendor protocol, so that package follows the `<vendor>_<series-or-protocol>` naming rule.

Note: Encoder feedback (motor → controller direction) is in the separate [hils_bridge_encoder](../hils_bridge_encoder/) category, since input and output sides of the loop should not share a payload.
