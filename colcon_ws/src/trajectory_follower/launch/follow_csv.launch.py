from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('trajectory_file', description='Absolute path to the NED CSV trajectory'),
        DeclareLaunchArgument('px4_namespace', default_value='/px4_100'),
        DeclareLaunchArgument('target_system', default_value='100'),
        DeclareLaunchArgument('publish_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('max_start_distance_m', default_value='0.5'),
        DeclareLaunchArgument('max_start_yaw_error_rad', default_value='0.5'),
        DeclareLaunchArgument('land_on_complete', default_value='false'),
        Node(
            package='trajectory_follower',
            executable='follow_csv',
            name='trajectory_follower',
            output='screen',
            parameters=[{
                'trajectory_file': LaunchConfiguration('trajectory_file'),
                'px4_namespace': LaunchConfiguration('px4_namespace'),
                'target_system': ParameterValue(LaunchConfiguration('target_system'), value_type=int),
                'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
                'max_start_distance_m': ParameterValue(
                    LaunchConfiguration('max_start_distance_m'), value_type=float),
                'max_start_yaw_error_rad': ParameterValue(
                    LaunchConfiguration('max_start_yaw_error_rad'), value_type=float),
                'land_on_complete': ParameterValue(
                    LaunchConfiguration('land_on_complete'), value_type=bool),
            }],
        ),
    ])
