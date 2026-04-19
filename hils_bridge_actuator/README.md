# hils_bridge_actuator

Actuator command HILS bridges. Each sub-package converts simulator command messages into the actuator's wire signal so the real PC's controller code is exercised end-to-end.

| Package | Output Signal | Firmware |
|---------|---------------|----------|
| [hils_bridge_actuator_servo_pwm](hils_bridge_actuator_servo_pwm/) | RC servo PWM (50 Hz, 500–2500 us) | rp2040_actuator_servo_pwm |

PWM with 50 Hz / 1–2 ms pulses is the de-facto standard for RC servo control across virtually all vendors, so `hils_bridge_actuator_servo_pwm` is named for the protocol rather than a specific vendor.

Note: Encoder feedback (motor → controller direction) is in the separate [hils_bridge_encoder](../hils_bridge_encoder/) category, since input and output sides of the loop should not share a payload.
