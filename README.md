**English** | [日本語](README-ja.md)

# ros2_hils_bridge

ROS 2 Hardware-in-the-Loop Simulation (HILS) bridge packages.

Converts ROS topics from simulators (Gazebo / Unity / Isaac Sim) into device-native protocols, enabling **end-to-end testing of real sensor drivers over physical communication paths** -- affordably.

## Concept

Software-in-the-Loop (SILS) publishes directly to ROS topics, so the actual sensor drivers and physical interfaces (UDP, UART, USB, I2C, etc.) are never exercised. These packages bridge that gap: the real-hardware driver sees exactly what it would see from a real sensor.

```
Simulation PC                                    Robot PC
┌─────────────┐     ┌──────────────────┐
│  Simulator   │────>│ hils_bridge_*    │
│  (PointCloud2│     │                  │──USB-LAN──> LiDAR driver
│   Image      │     │  Protocol conv.  │──FT234X──> GPS/IMU driver
│   NavSatFix  │     │                  │──RP2040──> USB camera driver
│   JointState)│     └──────────────────┘
└─────────────┘
```

## Naming Convention

Packages follow a **two-level naming pattern**:

```
hils_bridge_<sensor_type>/hils_bridge_<sensor_type>_<protocol_or_vendor_series>/
```

- **`<sensor_type>`** — physical/functional category: `lidar`, `camera`, `gps`, `imu`, `actuator`, `encoder`, `can`
- **`<protocol_or_vendor_series>`** — chosen by the following rule:
  - **Industry-standard protocol** (UVC, NMEA0183, PWM, quadrature, …): use the protocol name itself, signaling that any vendor's device with that protocol works (e.g. `hils_bridge_camera_uvc`, `hils_bridge_gps_nmea0183`)
  - **Vendor-specific protocol**: use `<vendor>_<series>` so the actual scope is unambiguous (e.g. `hils_bridge_lidar_livox_mid360`, `hils_bridge_imu_witmotion_wt901`). Even within one vendor, different generations or series often break wire-compatibility — always include the series

The same convention applies to the matching firmware under `firmware/rp2040_<sensor_type>_<protocol_or_vendor_series>/`.

## Packages

### Core

| Package | Description | Status |
|---------|-------------|--------|
| `hils_bridge_base` | Shared utilities (frame protocol, network, base classes) | Done |
| `hils_bringup` | Launch orchestration | Stub |

### LiDAR (USB-Ethernet adapter + pure software, no MCU needed)

| Package | Target Device | Real Driver | Status |
|---------|--------------|-------------|--------|
| `hils_bridge_lidar_livox_mid360` | Livox Mid-360 | livox_ros_driver2 | Done, **verified** |
| `hils_bridge_lidar_velodyne_vlp16` | Velodyne VLP-16 | velodyne_driver | Done, **verified** |
| `hils_bridge_lidar_ouster_os1` | Ouster OS1 | ouster_ros | Done, **verified**[^ouster-note] |

[^ouster-note]: For Ouster, the emulator exposes HTTP REST API on port 80, so Docker needs `sysctls: net.ipv4.ip_unprivileged_port_start=80`. See the [verification guide](../docs/hils_verification_guide.md#13-ouster-os1) for details.

### Camera (RP2040 x 2)

| Package | Target Device | Real Driver | Status |
|---------|--------------|-------------|--------|
| `hils_bridge_camera_uvc` | Any UVC USB camera (MJPEG) | usb_cam / cv_camera | Done, **verified** |

### GPS (FT234X cross-connection, no MCU needed)

| Package | Target Device | Real Driver | Status |
|---------|--------------|-------------|--------|
| `hils_bridge_gps_nmea0183` | Any NMEA 0183 GPS receiver | nmea_navsat_driver | Done, **verified** |

### IMU

| Package | Target Device | Real Driver | Interface | Status |
|---------|--------------|-------------|-----------|--------|
| `hils_bridge_imu_witmotion_wt901` | Witmotion WT901 (binary) | witmotion_ros (ElettraSciComp) | Serial (FT234X) | Done, **verified** |
| `hils_bridge_imu_invensense_mpu6050` | InvenSense MPU-6050 (register map) | (any I2C master) | I2C slave (RP2040) | Done, unverified |

### Actuator

| Package | Target Device | Output | Firmware | Status |
|---------|--------------|--------|----------|--------|
| `hils_bridge_actuator_servo_pwm` | RC servo (any vendor) | PWM 50Hz, 500–2500us pulse | rp2040_actuator_servo_pwm | Done, unverified |

### Encoder

| Package | Output Signal | Firmware | Status |
|---------|---------------|----------|--------|
| `hils_bridge_encoder_quadrature` | A/B quadrature pulses | rp2040_encoder_quadrature | Done, unverified |

### Future Expansion (placeholders)

| Directory | Planned Devices |
|-----------|----------------|
| `hils_bridge_can` | CAN bus motor drivers (CANopen, vendor-specific) |

## Required Hardware

| Purpose | Hardware | Approx. Cost | Notes |
|---------|----------|-------------|-------|
| LiDAR emulation | USB-Ethernet adapter | ~$7 / unit | No MCU needed |
| Serial sensors (GPS, IMU) | FT234X x 2 (Akizuki 108461) | ~$6 / pair | TX/RX cross-connection |
| UVC camera | Raspberry Pi Pico H x 2 | ~$13 | Firmware required |
| RC servo PWM | Raspberry Pi Pico H x 1 | ~$7 | Firmware required |
| Quadrature encoder | Raspberry Pi Pico H x 1 | ~$7 | Firmware required |
| I2C sensor | Raspberry Pi Pico H x 1 | ~$7 | Firmware required |

Minimum setup (LiDAR + camera) costs around $20.

## Build

```bash
cd ~/colcon_ws/src
git clone https://github.com/<your-org>/ros2_hils_bridge.git

cd ~/colcon_ws
colcon build
source install/setup.bash
```

### Dependencies

- ROS 2 Humble / Jazzy
- `sensor_msgs`, `geometry_msgs`, `cv_bridge`
- `python3-serial`, `python3-opencv`, `python3-numpy`

## Usage (example: Livox Mid-360)

```bash
# Simulation PC: assign LiDAR IP to a USB-Ethernet adapter
sudo ip addr add 192.168.1.12/24 dev eth1

# Launch emulator
ros2 launch hils_bridge_lidar_livox_mid360 livox_emulator.launch.py \
  network_interface:=eth1 \
  host_ip:=192.168.1.5 \
  pointcloud_topic:=/livox/lidar

# Robot PC: launch livox_ros_driver2 as usual
# Simulated point clouds are delivered through the real driver
```

## Firmware

UVC camera, PWM servo, and I2C sensor emulation require RP2040 firmware. Firmware is managed in the parent repository [ros_hardware_in_the_loop_system](https://github.com/<your-org>/ros_hardware_in_the_loop_system) under `firmware/`.

## License

MIT
