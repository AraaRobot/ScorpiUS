import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_joy = Node(
        package="scorpius_joy",
        namespace="/scorpius/joy",
        executable="joy",
        name="joy"
    )
    ld.add_action(node_joy)
    return ld