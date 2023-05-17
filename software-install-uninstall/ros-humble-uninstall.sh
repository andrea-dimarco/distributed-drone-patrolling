#!/bin/bash

sudo apt remove ~nros-humble-* && sudo apt autoremove

sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
# Consider upgrading for packages previously shadowed.
sudo apt upgrade
