"""Launch file for pure software Hokuyo YVT-35LX (VSSP 2.1) emulator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pointcloud_topic', default_value='/sim_points',
                              description='PointCloud2 topic from simulation'),
        DeclareLaunchArgument('tcp_port', default_value='10940',
                              description='VSSP TCP port'),
        DeclareLaunchArgument('scan_rate_hz', default_value='20.0',
                              description='Frames per second (datasheet: 20 Hz)'),
        DeclareLaunchArgument('horizontal_fov_deg', default_value='210.0',
                              description='Horizontal field of view'),
        DeclareLaunchArgument('vertical_fov_min_deg', default_value='-5.0',
                              description='Lower vertical scan angle'),
        DeclareLaunchArgument('vertical_fov_max_deg', default_value='35.0',
                              description='Upper vertical scan angle'),
        DeclareLaunchArgument('measured_lines', default_value='35',
                              description='Lines carrying measurements per '
                                          'frame (35 x 74 = 2590 points)'),
        DeclareLaunchArgument('range_noise_sigma_m', default_value='0.0',
                              description='Range noise std-dev in m (0 = off; '
                                          'leave off if the simulation already '
                                          'models sensor noise)'),
        DeclareLaunchArgument('dropout_probability', default_value='0.0',
                              description='Probability a spot reports no echo'),
        DeclareLaunchArgument('noise_seed', default_value='0',
                              description='Seed for the noise generator'),
        DeclareLaunchArgument('serial_number', default_value='H0000001',
                              description='Emulated serial number'),
        DeclareLaunchArgument('device_ip', default_value='192.168.0.10',
                              description='IP assigned to the network interface (emulated YVT-35LX)'),
        DeclareLaunchArgument('host_ip', default_value='192.168.0.15',
                              description='Robot PC IP (urg3d_node2)'),
        DeclareLaunchArgument('network_interface', default_value='',
                              description='Network interface (e.g. "eth1"). Empty = auto from device_ip'),

        Node(
            package='hils_bridge_lidar_hokuyo_yvt35lx',
            executable='yvt35lx_emulator_node',
            name='hils_yvt35lx_emulator',
            parameters=[{
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'tcp_port': LaunchConfiguration('tcp_port'),
                'scan_rate_hz': LaunchConfiguration('scan_rate_hz'),
                'horizontal_fov_deg': LaunchConfiguration('horizontal_fov_deg'),
                'vertical_fov_min_deg': LaunchConfiguration('vertical_fov_min_deg'),
                'vertical_fov_max_deg': LaunchConfiguration('vertical_fov_max_deg'),
                # Typed explicitly: "0" on the command line would
                # otherwise be inferred as an integer and rejected by
                # these double parameters.
                'measured_lines': ParameterValue(
                    LaunchConfiguration('measured_lines'), value_type=int),
                'range_noise_sigma_m': ParameterValue(
                    LaunchConfiguration('range_noise_sigma_m'),
                    value_type=float),
                'dropout_probability': ParameterValue(
                    LaunchConfiguration('dropout_probability'),
                    value_type=float),
                'noise_seed': ParameterValue(
                    LaunchConfiguration('noise_seed'), value_type=int),
                'serial_number': ParameterValue(
                    LaunchConfiguration('serial_number'), value_type=str),
                'device_ip': LaunchConfiguration('device_ip'),
                'host_ip': LaunchConfiguration('host_ip'),
                'network_interface': LaunchConfiguration('network_interface'),
            }],
            output='screen',
        ),
    ])
