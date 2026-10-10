from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch',
            'rs_launch.py',
        ])),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.color_profile': '640x480x30',
            'depth_module.depth_profile': '640x480x30',
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'pointcloud.enable': 'false',
        }.items(),
    )

    crop = Node(
        package='camera_448',
        executable='center_crop',
        name='camera_448_center_crop',
        output='screen',
        parameters=[{
            'crop_size': 448,
            'jpeg_quality': 90,
            'depth_png_level': 3,
        }],
    )

    return LaunchDescription([realsense, crop])
