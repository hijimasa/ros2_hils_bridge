# hils_bridge_imu

IMU HILS bridges. Each sub-package converts simulator Imu into the device's vendor-specific wire protocol so the real driver runs unmodified.

| Package | Target Device | Real Driver | Interface |
|---------|--------------|-------------|-----------|
| [hils_bridge_imu_witmotion_wt901](hils_bridge_imu_witmotion_wt901/) | Witmotion WT901 (binary 0x51/0x52/0x53/0x59) | witmotion_ros (ElettraSciComp) | Serial (FT234X cross-connect) |
| [hils_bridge_imu_invensense_mpu6050](hils_bridge_imu_invensense_mpu6050/) | InvenSense MPU-6050 (register map) | (any I2C master, e.g. mpu6050_driver) | I2C slave (RP2040) |

Vendor-specific protocols are not interchangeable between IMU series — `hils_bridge_imu_witmotion_wt901` will not drive a JY901 or HWT905 reliably. Add new sub-packages following the `hils_bridge_imu_<vendor>_<series>` convention.
