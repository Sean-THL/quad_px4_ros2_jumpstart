from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from trajectory_follower.live_trajectory import TrajectoryConfig

# FLOAT_PARAMETERS = {
#         'publish_rate_hz': '50.0',
#         'takeoff_height_m': '1.0',
#         'takeoff_duration_s': '3.0',
#         'flight_duration_s': '12.0',
#         'hold_duration_s': '2.0',
#         'north_radius_m': '1.0',
#         'east_radius_m': '0.6',
#         'vertical_amplitude_m': '0.15',
# }


# Keep launch defaults in sync with the live node and MeshCat preview.
TRAJECTORY_DEFAULTS = TrajectoryConfig()
FLOAT_PARAMETERS = {'publish_rate_hz': 50.0}
FLOAT_PARAMETERS.update({
    name: getattr(TRAJECTORY_DEFAULTS, name)
    for name in TrajectoryConfig.__dataclass_fields__
})


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('px4_namespace', default_value='/px4_100'),
        DeclareLaunchArgument('target_system', default_value='100'),
    ]
    arguments.extend(DeclareLaunchArgument(name, default_value=str(value))
                     for name, value in FLOAT_PARAMETERS.items())
    parameters = {
        'px4_namespace': LaunchConfiguration('px4_namespace'),
        'target_system': ParameterValue(
            LaunchConfiguration('target_system'), value_type=int),
    }
    parameters.update({
        name: ParameterValue(LaunchConfiguration(name), value_type=float)
        for name in FLOAT_PARAMETERS})
    arguments.append(Node(
        package='trajectory_follower',
        executable='follow_live',
        name='live_trajectory_follower',
        output='screen',
        parameters=[parameters],
    ))
    return LaunchDescription(arguments)
