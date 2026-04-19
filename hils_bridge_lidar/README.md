# hils_bridge_lidar

3D LiDAR HILS bridges. Each sub-package converts simulator PointCloud2 (and optionally Imu) into the vendor's native UDP wire protocol so the real device driver runs unmodified.

| Package | Target Device | Real Driver |
|---------|--------------|-------------|
| [hils_bridge_lidar_livox_mid360](hils_bridge_lidar_livox_mid360/) | Livox Mid-360 | livox_ros_driver2 |
| [hils_bridge_lidar_velodyne_vlp16](hils_bridge_lidar_velodyne_vlp16/) | Velodyne VLP-16 | velodyne_driver |
| [hils_bridge_lidar_ouster_os1](hils_bridge_lidar_ouster_os1/) | Ouster OS1 | ouster_ros |

All three implementations are pure Python — no microcontroller required. Bind the emulator to a USB-Ethernet adapter that holds the LiDAR's IP and the real driver sees a real LiDAR.
