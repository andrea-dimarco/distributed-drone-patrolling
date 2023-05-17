#!/bin/bash

# ROS dependencies
sudo apt install python3-colcon-common-extensions 
rosdep install -i --from-path src --rosdistro humble -y 

# GAZEBO - ROS interface
export ROS_DISTRO="humble"
sudo apt-get install ros-${ROS_DISTRO}-ros-gz 
