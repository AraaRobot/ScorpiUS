import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_media = Node(
        package="scorpius_media",
        namespace="/scorpius/media",
        executable="media",
        name="media"
    )
    ld.add_action(node_media)
    return ld