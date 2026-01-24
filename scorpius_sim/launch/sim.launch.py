import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_sim = Node(
        package="rover_sim",
        namespace="/rover/sim",
        executable="sim",
        name="sim"
    )
    ld.add_action(node_sim)
    return ld