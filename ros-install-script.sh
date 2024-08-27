#!/bin/bash

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and upgrade
sudo apt update -y
sudo apt upgrade -y

# Install ROS 2
sudo apt install ros-jazzy-desktop -y

source /opt/ros/jazzy/setup.bash

echo "/opt/ros/jazzy/setup.bash" >> ~/.bashrc