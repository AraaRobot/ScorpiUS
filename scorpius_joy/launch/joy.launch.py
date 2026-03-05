import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node_joy_main = Node(
        package="joy",
        namespace="/base/joy",
        executable="joy_node",
        name="main_joy_node",
        parameters=[{"autorepeat_rate": 20.0,
        # PS5 controller
        # "device_name": "DualSense Wireless Controller",
        "device_name": "Sony Interactive Entertainment DualSense Wireless Controller",
        # PS4 controller
        # "device_name": "Wireless Controller",
        #  "device_name": "Sony Interactive Entertainment Wireless Controller"}],
        "deadzone": 0.0}],
        remappings=[("joy", "main_raw")]
    )

    node_joy = Node(
        package="scorpius_joy",
        namespace="/base/joy",
        executable="joy_formator",
        name="joy_formator",
        remappings=[("raw/joy", "main_raw")]
    )

    ld.add_action(node_joy_main)
    ld.add_action(node_joy)
    return ld