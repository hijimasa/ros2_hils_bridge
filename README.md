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

## Packages

### Core

| Package | Description | Status |
|---------|-------------|--------|
| `hils_bridge_base` | Shared utilities (frame protocol, network, base classes) | Done |
| `hils_bringup` | Launch orchestration | Stub |

### LiDAR (USB-Ethernet adapter + pure software, no MCU needed)

| Package | Target Device | Real Driver | Status |
|---------|--------------|-------------|--------|
| `hils_bridge_lidar_livox` | Livox Mid-360 | livox_ros_driver2 | Done, **verified** |
| `hils_bridge_lidar_velodyne` | Velodyne VLP-16 | velodyne_driver | Done, **verified** |
| `hils_bridge_lidar_ouster` | Ouster OS1 | ouster_ros | Done, unverified |

### Camera (RP2040 x 2)

| Package | Target Device | Real Driver | Status |
|---------|--------------|-------------|--------|
| `hils_bridge_camera_uvc` | USB Camera (UVC/MJPEG) | usb_cam / cv_camera | Done, **verified** |

### Serial Sensors (FT234X x 2 cross-connection, no MCU needed)

| Package | Target Device | Real Driver | Status |
|---------|--------------|-------------|--------|
| `hils_bridge_serial_gps` | GPS (NMEA 0183) | nmea_navsat_driver | Done, unverified |
| `hils_bridge_serial_imu` | IMU (Witmotion WT901) | witmotion_ros | Done, unverified |

### Actuators (RP2040 PIO)

| Package | Target Device | Output | Status |
|---------|--------------|--------|--------|
| `hils_bridge_actuator_pwm` | RC Servo + Encoder | PWM (50Hz) + A/B quadrature | Done, unverified |

### I2C/SPI Sensors (RP2040 I2C slave)

| Package | Target Device | Interface | Status |
|---------|--------------|-----------|--------|
| `hils_bridge_sensor_i2c` | MPU-6050 IMU | I2C slave (0x68) | Done, unverified |

### Future Expansion (placeholders)

| Directory | Planned Devices |
|-----------|----------------|
| `hils_bridge_can` | CAN bus motor drivers (CANopen) |
| `hils_bridge_encoder` | Rotary encoders |
| `hils_bridge_gps` | GPS receivers (vendor-specific) |
| `hils_bridge_imu` | IMUs (vendor-specific) |

## Required Hardware

| Purpose | Hardware | Approx. Cost | Notes |
|---------|----------|-------------|-------|
| LiDAR emulation | USB-Ethernet adapter | ~$7 / unit | No MCU needed |
| Serial sensors | FT234X x 2 (Akizuki 108461) | ~$6 / pair | TX/RX cross-connection |
| UVC camera | Raspberry Pi Pico H x 2 | ~$13 | Firmware required |
| PWM servo / encoder | Raspberry Pi Pico H x 1 | ~$7 | Firmware required |
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
ros2 launch hils_bridge_lidar_livox livox_emulator.launch.py \
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
