import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_controller_sim = Node(
        package="scorpius_sim",
        namespace="/scorpius/sim",
        executable="controller_sim",
        name="controller_sim"
    )
    ld.add_action(node_controller_sim)
    return ld