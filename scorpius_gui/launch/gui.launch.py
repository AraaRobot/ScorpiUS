import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_comm = Node(
        package="scorpius_gui",
        namespace="/scorpius/gui",
        executable="gui",
        name="gui"
    )
    ld.add_action(node_comm)
    return ld