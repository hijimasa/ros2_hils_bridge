# hils_bridge_lidar

LiDAR HILS bridges. Each sub-package converts simulator sensor topics (PointCloud2 / LaserScan, and optionally Imu) into the vendor's native wire protocol so the real device driver runs unmodified.

| Package | Target Device | Real Driver | Transport |
|---------|--------------|-------------|-----------|
| [hils_bridge_lidar_livox_mid360](hils_bridge_lidar_livox_mid360/) | Livox Mid-360 | livox_ros_driver2 | UDP |
| [hils_bridge_lidar_velodyne_vlp16](hils_bridge_lidar_velodyne_vlp16/) | Velodyne VLP-16 | velodyne_driver | UDP |
| [hils_bridge_lidar_ouster_os1](hils_bridge_lidar_ouster_os1/) | Ouster OS1 | ouster_ros | UDP + HTTP |
| [hils_bridge_lidar_hokuyo_yvt35lx](hils_bridge_lidar_hokuyo_yvt35lx/) | Hokuyo YVT-35LX (3D-URG) | urg3d_node2 | TCP (VSSP 2.1) |
| [hils_bridge_lidar_slamtec_rplidar_a](hils_bridge_lidar_slamtec_rplidar_a/) | SLAMTEC RPLIDAR (A-series serial protocol) | rplidar_ros | Serial (FT234X or PTY) |

All implementations are pure Python — no microcontroller required. For the network LiDARs, bind the emulator to a USB-Ethernet adapter that holds the LiDAR's IP and the real driver sees a real LiDAR; the RPLIDAR bridge uses an FT234X cross-connection (or a PTY for SILS) instead.
