from datetime import datetime

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from camera_448.flight_topics import (
    BASE_RECORD_TOPICS, DEPTH_COMPRESSED_TOPIC, DEPTH_RAW_TOPIC,
    RGB_COMPRESSED_TOPIC, RGB_RAW_TOPIC)


def _enabled(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ('true', 'false'):
        raise ValueError(f'{name} must be true or false, got {value!r}')
    return value == 'true'


def _make_recorder(context, bag_name):
    image_topics = []
    selections = (
        ('record_raw_rgb', RGB_RAW_TOPIC),
        ('record_compressed_rgb', RGB_COMPRESSED_TOPIC),
        ('record_raw_depth', DEPTH_RAW_TOPIC),
        ('record_compressed_depth', DEPTH_COMPRESSED_TOPIC),
    )
    for argument, topic in selections:
        if _enabled(context, argument):
            image_topics.append(topic)

    if not any(topic in image_topics for topic in
               (RGB_RAW_TOPIC, RGB_COMPRESSED_TOPIC)):
        raise ValueError('Enable at least one RGB recording format')
    if not any(topic in image_topics for topic in
               (DEPTH_RAW_TOPIC, DEPTH_COMPRESSED_TOPIC)):
        raise ValueError('Enable at least one depth recording format')

    output_path = PathJoinSubstitution([
        LaunchConfiguration('output_directory'), bag_name])
    return [ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '--storage', 'sqlite3',
            '-o', output_path, *image_topics, *BASE_RECORD_TOPICS,
        ],
        output='screen',
    )]


def generate_launch_description():
    bag_name = 'flight_rgbd_' + datetime.now().strftime('%Y%m%d_%H%M%S')
    start_px4_stack = LaunchConfiguration('start_px4_stack')

    px4_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('all_launch'), 'launch', 'px4.launch.py'
        ])),
        condition=IfCondition(start_px4_stack),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('camera_448'), 'launch', 'camera_448.launch.py'
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_directory', default_value='/home/colcon_ws/camera',
            description='Directory in which the timestamped bag is created'),
        DeclareLaunchArgument(
            'start_px4_stack', default_value='true',
            description='Start XRCE, MAVLink router, and Vicon bridges'),
        DeclareLaunchArgument(
            'record_raw_rgb', default_value='false',
            description='Record raw 448x448 RGB images'),
        DeclareLaunchArgument(
            'record_compressed_rgb', default_value='true',
            description='Record JPEG-compressed 448x448 RGB images'),
        DeclareLaunchArgument(
            'record_raw_depth', default_value='false',
            description='Record raw 16UC1 448x448 depth images'),
        DeclareLaunchArgument(
            'record_compressed_depth', default_value='true',
            description='Record lossless PNG-compressed 448x448 depth images'),
        px4_stack,
        # Prevent recorder-specific launch arguments from leaking into the
        # third-party RealSense launch file and generating false warnings.
        GroupAction(scoped=True, forwarding=False, actions=[camera]),
        # Give the camera and bridges time to advertise before rosbag subscribes.
        TimerAction(
            period=3.0,
            actions=[OpaqueFunction(
                function=_make_recorder,
                kwargs={'bag_name': bag_name})]),
    ])
