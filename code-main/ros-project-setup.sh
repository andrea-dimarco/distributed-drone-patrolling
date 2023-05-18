#!/bin/bash

sudo apt update

# env variables
export ROS_DISTRO="humble"
source /opt/ros/humble/setup.bash

# ROS dependencies
sudo apt install python3-colcon-common-extensions 
rosdep install -i --from-path src --rosdistro humble -y 
sudo rosdep init
rosdep update

# GAZEBO - ROS interface
sudo apt-get install ros-${ROS_DISTRO}-ros-gz 

# pip, just in case
sudo apt update
sudo apt install python3-pip
pip3 --version


# PyGUI
pip install dearpygui
sudo apt update

source /opt/ros/humble/setup.bash
colcon build
./start
