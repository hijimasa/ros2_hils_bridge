"""Launch file for the OrientalMotor ID-share motor bridge node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory(
        'hils_bridge_actuator_orientalmotor_id_share')
    default_params_file = os.path.join(pkg_dir, 'config', 'default_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file,
                              description='Full parameter file '
                                          '(joint_names, gear_ratios, ...)'),
        DeclareLaunchArgument('serial_port',
                              default_value='/tmp/ttyUSB_BlvrMotor',
                              description='Serial device, or the symlink '
                                          'path to create when create_pty'),
        DeclareLaunchArgument('create_pty', default_value='true',
                              description='Create a virtual serial device '
                                          '(PTY) at serial_port (SILS)'),
        DeclareLaunchArgument('baudrate', default_value='230400',
                              description='Serial baudrate (match the '
                                          'daemon; meaningless on a PTY)'),
        DeclareLaunchArgument('motor_types', default_value='BLVR:400,BLVR:400',
                              description='Comma-separated TYPE[:KEY] list, '
                                          'e.g. "BLVR:100,AZ:60"'),

        Node(
            package='hils_bridge_actuator_orientalmotor_id_share',
            executable='oms_bridge_node',
            name='hils_oms_bridge',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'serial_port': LaunchConfiguration('serial_port'),
                    'create_pty': ParameterValue(
                        LaunchConfiguration('create_pty'), value_type=bool),
                    'baudrate': ParameterValue(
                        LaunchConfiguration('baudrate'), value_type=int),
                    'motor_types': ParameterValue(
                        LaunchConfiguration('motor_types'), value_type=str),
                },
            ],
            output='screen',
        ),
    ])
