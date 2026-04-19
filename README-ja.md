[English](README.md) | **日本語**

# ros2_hils_bridge

ROS 2 Hardware-in-the-Loop Simulation (HILS) ブリッジパッケージ群。

シミュレータ（Gazebo / Unity / Isaac Sim）の ROS トピックを実機センサのプロトコルに変換し、**実機のドライバノードを物理通信経路を含めてテスト**するための HILS 環境を安価に構築する。

## コンセプト

ソフトウェアシミュレーション (SILS) では ROS トピックに直接パブリッシュするため、実機のセンサドライバや物理通信経路（UDP, UART, USB, I2C 等）がテストされない。本パッケージ群は、シミュレータの出力をセンサ固有のプロトコルに変換し、実機ドライバが「本物のセンサが接続されている」と認識する状態を作り出す。

```
シミュレーションPC                                    実機PC (Robot)
┌─────────────┐     ┌──────────────────┐
│  Simulator   │────>│ hils_bridge_*    │
│  (PointCloud2│     │                  │──USB-LAN──> LiDARドライバ
│   Image      │     │  プロトコル変換    │──FT234X──> GPS/IMUドライバ
│   NavSatFix  │     │                  │──RP2040──> USBカメラドライバ
│   JointState)│     └──────────────────┘
└─────────────┘
```

## パッケージ一覧

### 共通基盤

| パッケージ | 説明 | 状態 |
|-----------|------|------|
| `hils_bridge_base` | 共通ユーティリティ（フレームプロトコル、ネットワーク、基底クラス） | 実装済 |
| `hils_bringup` | 起動ファイルの統合 | スタブ |

### LiDAR（USB-LAN アダプタ + 純ソフトウェア、マイコン不要）

| パッケージ | 対象デバイス | 実機ドライバ | 状態 |
|-----------|------------|------------|------|
| `hils_bridge_lidar_livox` | Livox Mid-360 | livox_ros_driver2 | 実装済・動作確認済 |
| `hils_bridge_lidar_velodyne` | Velodyne VLP-16 | velodyne_driver | 実装済・**動作確認済** |
| `hils_bridge_lidar_ouster` | Ouster OS1 | ouster_ros | 実装済・未検証 |

### カメラ（RP2040 x 2）

| パッケージ | 対象デバイス | 実機ドライバ | 状態 |
|-----------|------------|------------|------|
| `hils_bridge_camera_uvc` | USB カメラ (UVC/MJPEG) | usb_cam / cv_camera | 実装済・動作確認済 |

### シリアルセンサ（FT234X x 2 クロス接続、マイコン不要）

| パッケージ | 対象デバイス | 実機ドライバ | 状態 |
|-----------|------------|------------|------|
| `hils_bridge_serial_gps` | GPS (NMEA 0183) | nmea_navsat_driver | 実装済・未検証 |
| `hils_bridge_serial_imu` | IMU (Witmotion WT901) | witmotion_ros | 実装済・未検証 |

### アクチュエータ（RP2040 PIO）

| パッケージ | 対象デバイス | 出力 | 状態 |
|-----------|------------|------|------|
| `hils_bridge_actuator_pwm` | RC サーボ + エンコーダ | PWM (50Hz) + A/B 相 | 実装済・未検証 |

### I2C/SPI センサ（RP2040 I2C スレーブ）

| パッケージ | 対象デバイス | インターフェース | 状態 |
|-----------|------------|----------------|------|
| `hils_bridge_sensor_i2c` | MPU-6050 IMU | I2C スレーブ (0x68) | 実装済・未検証 |

### 将来の拡張（プレースホルダ）

| ディレクトリ | 想定デバイス |
|------------|------------|
| `hils_bridge_can` | CAN バスモータドライバ (CANopen) |
| `hils_bridge_encoder` | ロータリエンコーダ |
| `hils_bridge_gps` | GPS 受信機（メーカー別パッケージ） |
| `hils_bridge_imu` | IMU（メーカー別パッケージ） |

## 必要ハードウェア

| 用途 | ハードウェア | 概算コスト | 備考 |
|------|------------|----------|------|
| LiDAR エミュレーション | USB-LAN アダプタ | ~1,000 円/台 | マイコン不要 |
| シリアルセンサ | FT234X x 2 (秋月 108461) | ~800 円/組 | TX/RX クロス接続 |
| UVC カメラ | Raspberry Pi Pico H x 2 | ~1,840 円 | 要ファームウェア |
| PWM サーボ / エンコーダ | Raspberry Pi Pico H x 1 | ~920 円 | 要ファームウェア |
| I2C センサ | Raspberry Pi Pico H x 1 | ~920 円 | 要ファームウェア |

最小構成（LiDAR + カメラ）は約 3,000 円で構築可能。

## ビルド

```bash
cd ~/colcon_ws/src
git clone https://github.com/<your-org>/ros2_hils_bridge.git

cd ~/colcon_ws
colcon build
source install/setup.bash
```

### 依存パッケージ

- ROS 2 Humble
- `sensor_msgs`, `geometry_msgs`, `cv_bridge`
- `python3-serial`, `python3-opencv`, `python3-numpy`

## 使い方（例: Livox Mid-360）

```bash
# シミュレーションPC: USB-LANアダプタにLiDAR IPを割当
sudo ip addr add 192.168.1.12/24 dev eth1

# エミュレータ起動
ros2 launch hils_bridge_lidar_livox livox_emulator.launch.py \
  network_interface:=eth1 \
  host_ip:=192.168.1.5 \
  pointcloud_topic:=/livox/lidar

# 実機PC: livox_ros_driver2 を通常通り起動
# シミュレータの点群が実機ドライバ経由で配信される
```

## ファームウェア

UVC カメラ・PWM サーボ・I2C センサのエミュレーションには RP2040 ファームウェアが必要。ファームウェアは親リポジトリ [ros_hardware_in_the_loop_system](https://github.com/<your-org>/ros_hardware_in_the_loop_system) の `firmware/` ディレクトリで管理している。

## ライセンス

MIT
