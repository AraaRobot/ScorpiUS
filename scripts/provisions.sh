# !/bin/bash

set -e

echo "Installing ScorpiUS project dependencies..."

echo "=== Updating apt ==="
sudo apt update
echo -e "\e[0;32m[OK]\e[0m"

echo "=== Installing dependencies ==="
sudo apt install -y ros-dev-tools
sudo apt install -y ros-jazzy-desktop
sudo apt install -y ros-jazzy-ros-base
sudo apt install -y ros-jazzy-joy
sudo apt install -y ros-jazzy-rmw-cyclonedds-cpp
sudo apt install -y qt6-base-dev
sudo apt install -y qt6-tools-dev
sudo apt install -y qt6-webengine-dev
sudo apt install -y qt6-webengine-dev-tools
sudo apt install -y libqt6svg6-dev
sudo apt install -y libqt6webenginecore6-bin
sudo apt install -y sl
sudo apt install -y clang-format
sudo apt install -y python3
sudo apt install -y python3-venv
echo -e "\e[0;32m[OK]\e[0m"