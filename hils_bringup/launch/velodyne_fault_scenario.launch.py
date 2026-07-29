"""Launch the Velodyne emulator together with the fault scenario runner.

Usage:
    ros2 launch hils_bringup velodyne_fault_scenario.launch.py \
        device_ip:=192.168.1.201 host_ip:=192.168.1.100 \
        scenario:=<path or installed scenario name>

`scenario` accepts an absolute path, or a file name under this
package's scenarios/lidar/ directory (e.g. velodyne_power_loss_001.yaml).
The scenario starts automatically once the emulator's fault services
are available; set autostart:=false to start manually via
/hils_scenario_runner/start_scenario.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_scenario(context, *args, **kwargs):
    scenario = LaunchConfiguration('scenario').perform(context)
    if scenario and not os.path.isabs(scenario):
        scenario = os.path.join(
            get_package_share_directory('hils_bringup'),
            'scenarios', 'lidar', scenario)
    runner = Node(
        package='hils_bridge_base',
        executable='scenario_runner',
        name='hils_scenario_runner',
        output='screen',
        parameters=[{
            'scenario_file': scenario,
            'autostart': LaunchConfiguration('autostart'),
        }],
    )
    return [runner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device_ip', default_value='192.168.1.201',
                              description='IP emulating the VLP-16'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.100',
                              description='Robot PC IP receiving packets'),
        DeclareLaunchArgument('pointcloud_topic',
                              default_value='/velodyne_points',
                              description='PointCloud2 input topic'),
        DeclareLaunchArgument(
            'scenario', default_value='velodyne_power_loss_001.yaml',
            description='Scenario path or name under scenarios/lidar/'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Start scenario automatically'),
        Node(
            package='hils_bridge_lidar_velodyne_vlp16',
            executable='velodyne_emulator_node',
            name='hils_velodyne_emulator',
            output='screen',
            parameters=[{
                'device_ip': LaunchConfiguration('device_ip'),
                'host_ip': LaunchConfiguration('host_ip'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            }],
        ),
        OpaqueFunction(function=_resolve_scenario),
    ])
