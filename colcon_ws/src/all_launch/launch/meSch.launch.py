import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    # launch the candidate trajectory node 
    cand_traj_node = Node(
        name="candidate_trajectory",
        executable="eware_CandTraj_node",
        package="eware"
        )

    # launch the setpoint publisher 
    setpoint_pub_node = Node(
        name="setpoint_pub_node",
        executable="eware_SpPub_node",
        package="eware"
        )

    nodes = [
            cand_traj_node,
            setpoint_pub_node
            ]

    return LaunchDescription(nodes)