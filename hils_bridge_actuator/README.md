# hils_bridge_actuator

Actuator-path HILS bridges. Each sub-package captures the wire signal that the real robot controller emits toward its actuators and republishes the measurement to ROS so the simulator / test harness can evaluate what the controller is commanding.

| Package | Captured Signal | Firmware |
|---------|-----------------|----------|
| [hils_bridge_actuator_servo_pwm](hils_bridge_actuator_servo_pwm/) | RC servo PWM (50 Hz, 500–2500 us pulses) | rp2040_actuator_servo_pwm |

PWM with 50 Hz / 1–2 ms pulses is the de-facto standard for RC servo control across virtually all vendors, so `hils_bridge_actuator_servo_pwm` is named for the protocol rather than a specific vendor.

Note: Encoder feedback (motor → controller direction) is in the separate [hils_bridge_encoder](../hils_bridge_encoder/) category, since input and output sides of the loop should not share a payload.
