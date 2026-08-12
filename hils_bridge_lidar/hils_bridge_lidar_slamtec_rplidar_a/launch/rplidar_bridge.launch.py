"""Launch file for the SLAMTEC RPLIDAR A-series bridge node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory(
        'hils_bridge_lidar_slamtec_rplidar_a')
    default_params_file = os.path.join(pkg_dir, 'config', 'default_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='Full parameter file '
                                          '(device info, ...)'),
        DeclareLaunchArgument('serial_port',
                              default_value='/tmp/rplidar_front',
                              description='Serial device, or the symlink '
                                          'path to create when create_pty'),
        DeclareLaunchArgument('create_pty', default_value='true',
                              description='Create a virtual serial device '
                                          '(PTY) at serial_port (SILS)'),
        DeclareLaunchArgument('baudrate', default_value='1000000',
                              description='Serial baudrate (match the '
                                          'driver; meaningless on a PTY)'),
        DeclareLaunchArgument('scan_topic', default_value='/scan',
                              description='LaserScan topic from the '
                                          'simulator'),

        Node(
            package='hils_bridge_lidar_slamtec_rplidar_a',
            executable='rplidar_bridge_node',
            name='hils_rplidar_bridge',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'serial_port': LaunchConfiguration('serial_port'),
                    'create_pty': ParameterValue(
                        LaunchConfiguration('create_pty'), value_type=bool),
                    'baudrate': ParameterValue(
                        LaunchConfiguration('baudrate'), value_type=int),
                    'scan_topic': LaunchConfiguration('scan_topic'),
                },
            ],
            output='screen',
        ),
    ])
