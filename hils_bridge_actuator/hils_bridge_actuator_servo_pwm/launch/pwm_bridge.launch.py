"""Launch file for the PWM servo capture bridge node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hils_bridge_actuator_servo_pwm')
    default_params_file = os.path.join(pkg_dir, 'config', 'default_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Serial port for RP2040 USB CDC'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='Serial baudrate'),
        DeclareLaunchArgument('joint_state_topic',
                              default_value='/servo_pwm/joint_states',
                              description='Output JointState topic (measured)'),
        DeclareLaunchArgument('pulse_topic',
                              default_value='/servo_pwm/pulses_us',
                              description='Output raw pulse-width topic (us)'),
        DeclareLaunchArgument('servo_channels', default_value='4',
                              description='Number of servo channels to publish (1-4)'),
        DeclareLaunchArgument('servo_min_angle', default_value='-1.5708',
                              description='Angle mapped to servo_min_pulse_us (rad)'),
        DeclareLaunchArgument('servo_max_angle', default_value='1.5708',
                              description='Angle mapped to servo_max_pulse_us (rad)'),
        DeclareLaunchArgument('servo_min_pulse_us', default_value='500',
                              description='Pulse width at servo_min_angle (us)'),
        DeclareLaunchArgument('servo_max_pulse_us', default_value='2500',
                              description='Pulse width at servo_max_angle (us)'),

        Node(
            package='hils_bridge_actuator_servo_pwm',
            executable='pwm_bridge_node',
            name='hils_servo_pwm_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'pulse_topic': LaunchConfiguration('pulse_topic'),
                'servo_channels': LaunchConfiguration('servo_channels'),
                'servo_min_angle': LaunchConfiguration('servo_min_angle'),
                'servo_max_angle': LaunchConfiguration('servo_max_angle'),
                'servo_min_pulse_us': LaunchConfiguration('servo_min_pulse_us'),
                'servo_max_pulse_us': LaunchConfiguration('servo_max_pulse_us'),
            }],
            output='screen',
        ),
    ])
