import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_comm = Node(
        package="rover_comm",
        namespace="/rover/comm",
        executable="comm",
        name="comm"
    )
    ld.add_action(node_comm)
    return ld