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

## 命名規則

パッケージは **2 段階の命名パターン** に従う：

```
hils_bridge_<sensor_type>/hils_bridge_<sensor_type>_<protocol_or_vendor_series>/
```

- **`<sensor_type>`** — 物理／機能カテゴリ: `lidar`, `camera`, `gps`, `imu`, `actuator`, `encoder`, `can`
- **`<protocol_or_vendor_series>`** — 以下のルールで選ぶ：
  - **業界標準プロトコル** (UVC, NMEA0183, PWM, quadrature 等) → プロトコル名そのものを使う。任意のベンダーで使い回せることを示す (例: `hils_bridge_camera_uvc`, `hils_bridge_gps_nmea0183`)
  - **ベンダー固有プロトコル** → `<ベンダー>_<シリーズ>` 形式。実装範囲を曖昧にしない (例: `hils_bridge_lidar_livox_mid360`, `hils_bridge_imu_witmotion_wt901`)。同一ベンダー内でも世代やシリーズで非互換になりがちなので必ずシリーズ名まで含める

対応する firmware も同じ規則で `firmware/rp2040_<sensor_type>_<protocol_or_vendor_series>/` に配置する。

## パッケージ一覧

### 共通基盤

| パッケージ | 説明 | 状態 |
|-----------|------|------|
| `hils_bridge_base` | 共通ユーティリティ（フレームプロトコル、ネットワーク、基底クラス） | 実装済 |
| `hils_bringup` | 起動ファイルの統合 | スタブ |

### LiDAR（USB-LAN アダプタ + 純ソフトウェア、マイコン不要）

| パッケージ | 対象デバイス | 実機ドライバ | 状態 |
|-----------|------------|------------|------|
| `hils_bridge_lidar_livox_mid360` | Livox Mid-360 | livox_ros_driver2 | 実装済・**動作確認済** |
| `hils_bridge_lidar_velodyne_vlp16` | Velodyne VLP-16 | velodyne_driver | 実装済・**動作確認済** |
| `hils_bridge_lidar_ouster_os1` | Ouster OS1 | ouster_ros | 実装済・**動作確認済**（注意点あり[^ouster-note]） |

[^ouster-note]: Ouster はエミュレータ側で HTTP REST API (port 80) を提供するため Docker では `sysctls: net.ipv4.ip_unprivileged_port_start=80` が必要。詳細は [動作確認手順書](../docs/hils_verification_guide.md#13-ouster-os1) を参照。

### カメラ（RP2040 x 2）

| パッケージ | 対象デバイス | 実機ドライバ | 状態 |
|-----------|------------|------------|------|
| `hils_bridge_camera_uvc` | 任意の UVC USB カメラ (MJPEG) | usb_cam / cv_camera | 実装済・**動作確認済** |

### GPS（FT234X クロス接続、マイコン不要）

| パッケージ | 対象デバイス | 実機ドライバ | 状態 |
|-----------|------------|------------|------|
| `hils_bridge_gps_nmea0183` | 任意の NMEA 0183 GPS 受信機 | nmea_navsat_driver | 実装済・**動作確認済** |

### IMU

| パッケージ | 対象デバイス | 実機ドライバ | インターフェース | 状態 |
|-----------|------------|------------|----------------|------|
| `hils_bridge_imu_witmotion_wt901` | Witmotion WT901 (バイナリ) | witmotion_ros (ElettraSciComp) | シリアル (FT234X) | 実装済・**動作確認済** |
| `hils_bridge_imu_invensense_mpu6050` | InvenSense MPU-6050 (レジスタマップ) | (任意の I2C マスタ) | I2C スレーブ (RP2040) | 実装済・未検証 |

### アクチュエータ

| パッケージ | 対象デバイス | 出力 | ファームウェア | 状態 |
|-----------|------------|------|--------------|------|
| `hils_bridge_actuator_servo_pwm` | RC サーボ（任意のベンダー） | PWM 50Hz, 500–2500us パルス | rp2040_actuator_servo_pwm | 実装済・未検証 |

### エンコーダ

| パッケージ | 出力信号 | ファームウェア | 状態 |
|-----------|--------|--------------|------|
| `hils_bridge_encoder_quadrature` | A/B クワドラチャパルス | rp2040_encoder_quadrature | 実装済・未検証 |

### 将来の拡張（プレースホルダ）

| ディレクトリ | 想定デバイス |
|------------|------------|
| `hils_bridge_can` | CAN バスモータドライバ (CANopen, ベンダー固有) |

## 必要ハードウェア

| 用途 | ハードウェア | 概算コスト | 備考 |
|------|------------|----------|------|
| LiDAR エミュレーション | USB-LAN アダプタ | ~1,000 円/台 | マイコン不要 |
| シリアルセンサ (GPS, IMU) | FT234X x 2 (秋月 108461) | ~800 円/組 | TX/RX クロス接続 |
| UVC カメラ | Raspberry Pi Pico H x 2 | ~1,840 円 | 要ファームウェア |
| RC サーボ PWM | Raspberry Pi Pico H x 1 | ~920 円 | 要ファームウェア |
| クワドラチャエンコーダ | Raspberry Pi Pico H x 1 | ~920 円 | 要ファームウェア |
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
ros2 launch hils_bridge_lidar_livox_mid360 livox_emulator.launch.py \
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
