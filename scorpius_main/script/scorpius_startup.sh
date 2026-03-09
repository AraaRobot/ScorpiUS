#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash
source $HOME/scorpius_ws/install/local_setup.bash

ros2 launch scorpius_main main.launch.py