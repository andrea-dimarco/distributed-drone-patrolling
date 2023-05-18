#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build

. install/local_setup.bash

./start
