"""Launch file for quadrature encoder bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Serial port for RP2040 USB CDC'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='Serial baudrate'),
        DeclareLaunchArgument('max_hz', default_value='50.0',
                              description='Maximum encoder update rate'),
        DeclareLaunchArgument('joint_state_topic', default_value='/joint_states',
                              description='JointState topic from simulation'),
        DeclareLaunchArgument('encoder_channels', default_value='2',
                              description='Number of encoder channels (1-2)'),
        DeclareLaunchArgument('encoder_cpr', default_value='1000',
                              description='Encoder counts per revolution'),

        Node(
            package='hils_bridge_encoder_quadrature',
            executable='encoder_bridge_node',
            name='hils_encoder_quadrature_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'max_hz': LaunchConfiguration('max_hz'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'encoder_channels': LaunchConfiguration('encoder_channels'),
                'encoder_cpr': LaunchConfiguration('encoder_cpr'),
            }],
            output='screen',
        ),
    ])
