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
STRING_PARAMETERS = (
    'trajectory_type', 'yaw_mode', 'takeoff_profile',
    'offset_spiral_alignment_axis')
FLOAT_PARAMETERS = {
    'publish_rate_hz': 50.0,
    'activation_timeout_s': 10.0,
}
FLOAT_PARAMETERS.update({
    name: getattr(TRAJECTORY_DEFAULTS, name)
    for name in TrajectoryConfig.__dataclass_fields__
    if name not in STRING_PARAMETERS
})


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('px4_namespace', default_value='/px4_100'),
        DeclareLaunchArgument('target_system', default_value='100'),
        DeclareLaunchArgument(
            'allow_unknown_system_id', default_value='false',
            description=(
                'Allow VehicleStatus.system_id=0 only when MAV_SYS_ID has '
                'been verified independently')),
        DeclareLaunchArgument(
            'auto_arm_offboard', default_value='true',
            description=(
                'Send Arm and Offboard commands automatically; false waits '
                'for GUI control while streaming a hold setpoint')),
        DeclareLaunchArgument(
            'auto_command_interface', default_value='vehicle_command',
            description=(
                'vehicle_command or commander_set_state; use the latter '
                'with the custom simple_commander firmware')),
        DeclareLaunchArgument(
            'trajectory_type',
            default_value=TRAJECTORY_DEFAULTS.trajectory_type,
            description=(
                'figure_eight, lissajous_figure_eight, spiral, '
                'double_spiral, or offset_spiral')),
        DeclareLaunchArgument(
            'yaw_mode', default_value=TRAJECTORY_DEFAULTS.yaw_mode,
            description='fixed or velocity-aligned yaw'),
        DeclareLaunchArgument(
            'takeoff_profile',
            default_value=TRAJECTORY_DEFAULTS.takeoff_profile,
            description=(
                'smooth ramp or prearmed_hover target matching the RViz GUI')),
        DeclareLaunchArgument(
            'offset_spiral_alignment_axis',
            default_value=TRAJECTORY_DEFAULTS.offset_spiral_alignment_axis,
            description='x for plane x=x_align, y for plane y=y_align'),
    ]
    arguments.extend(DeclareLaunchArgument(name, default_value=str(value))
                     for name, value in FLOAT_PARAMETERS.items())
    parameters = {
        'px4_namespace': LaunchConfiguration('px4_namespace'),
        'target_system': ParameterValue(
            LaunchConfiguration('target_system'), value_type=int),
        'allow_unknown_system_id': ParameterValue(
            LaunchConfiguration('allow_unknown_system_id'), value_type=bool),
        'auto_arm_offboard': ParameterValue(
            LaunchConfiguration('auto_arm_offboard'), value_type=bool),
        'auto_command_interface': LaunchConfiguration(
            'auto_command_interface'),
        'trajectory_type': LaunchConfiguration('trajectory_type'),
        'yaw_mode': LaunchConfiguration('yaw_mode'),
        'takeoff_profile': LaunchConfiguration('takeoff_profile'),
        'offset_spiral_alignment_axis': LaunchConfiguration(
            'offset_spiral_alignment_axis'),
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
