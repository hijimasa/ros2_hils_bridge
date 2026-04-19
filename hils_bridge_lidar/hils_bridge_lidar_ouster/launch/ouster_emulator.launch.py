"""Launch file for pure software Ouster OS1 emulator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pointcloud_topic', default_value='/ouster/points',
                              description='PointCloud2 topic from simulation'),
        DeclareLaunchArgument('imu_topic', default_value='/ouster/imu',
                              description='Imu topic from simulation'),
        DeclareLaunchArgument('enable_imu', default_value='true',
                              description='Enable IMU packet output'),
        DeclareLaunchArgument('max_hz', default_value='10.0',
                              description='Max point cloud publish rate'),
        DeclareLaunchArgument('max_points_per_frame', default_value='30000',
                              description='Max points per frame'),
        DeclareLaunchArgument('downsample_mode', default_value='uniform',
                              description='Downsample: "uniform" or "near"'),
        DeclareLaunchArgument('lidar_mode', default_value='512x10',
                              description='Lidar mode string: 512x10, 1024x10, 2048x10'),
        DeclareLaunchArgument('n_channels', default_value='16',
                              description='Number of vertical channels (16, 32, 64)'),
        DeclareLaunchArgument('serial_number', default_value='992008000101',
                              description='Emulated LiDAR serial number'),
        DeclareLaunchArgument('elevation_filter', default_value='true',
                              description='Drop points outside sensor vertical FOV'),
        DeclareLaunchArgument('device_ip', default_value='192.168.1.100',
                              description='IP assigned to the network interface (emulated OS1)'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.5',
                              description='Robot PC IP (ouster_driver)'),
        DeclareLaunchArgument('network_interface', default_value='',
                              description='Network interface (e.g. "eth1"). Empty = auto from device_ip'),

        Node(
            package='hils_bridge_lidar_ouster',
            executable='ouster_emulator_node',
            name='hils_ouster_emulator',
            parameters=[{
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'enable_imu': LaunchConfiguration('enable_imu'),
                'max_hz': LaunchConfiguration('max_hz'),
                'max_points_per_frame': LaunchConfiguration('max_points_per_frame'),
                'downsample_mode': LaunchConfiguration('downsample_mode'),
                'lidar_mode': ParameterValue(
                    LaunchConfiguration('lidar_mode'), value_type=str),
                'n_channels': LaunchConfiguration('n_channels'),
                'serial_number': ParameterValue(
                    LaunchConfiguration('serial_number'), value_type=str),
                'elevation_filter': LaunchConfiguration('elevation_filter'),
                'device_ip': LaunchConfiguration('device_ip'),
                'host_ip': LaunchConfiguration('host_ip'),
                'network_interface': LaunchConfiguration('network_interface'),
            }],
            output='screen',
        ),
    ])
