# hils_bridge_can

CAN bus device HILS bridges. **Placeholder — no implementations yet.**

Planned sub-packages will follow the naming convention:

- `hils_bridge_can_canopen` for CANopen-protocol motor drivers / actuators (industry standard)
- `hils_bridge_can_<vendor>_<series>` for vendor-specific CAN device protocols

The corresponding firmware would live under `firmware/esp32_can_bridge/` since the ESP32-S3 has a built-in TWAI (CAN-compatible) peripheral.
