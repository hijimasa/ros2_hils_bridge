# hils_bridge_encoder

Encoder feedback HILS bridges. Each sub-package converts simulator joint state into the encoder's wire signal so the real PC's controller code (which reads encoder pulses to estimate motor position) can be tested.

| Package | Output Signal | Firmware |
|---------|---------------|----------|
| [hils_bridge_encoder_quadrature](hils_bridge_encoder_quadrature/) | A/B quadrature pulses on GPIO | rp2040_encoder_quadrature |

Quadrature A/B is the de-facto industry standard for incremental encoders, so `hils_bridge_encoder_quadrature` works with any controller that decodes A/B phase pulses. Vendor-specific feedback protocols (BiSS-C, EnDat, etc.) would live in their own sub-packages when implemented.

Note: Encoder emulation is intentionally separated from PWM servo emulation ([hils_bridge_actuator_servo_pwm](../hils_bridge_actuator/hils_bridge_actuator_servo_pwm/)) because they play different roles in the HILS loop — PWM is the controller's command output, while encoder pulses are the motor's feedback to the controller.
