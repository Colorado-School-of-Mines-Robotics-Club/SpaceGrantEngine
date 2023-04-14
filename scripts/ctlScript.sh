#!/bin/bash

# NEED THE ENVIRONMENT VARIABLES TO GET THE RIGHT INFO
# SEE THE TOP LINK IF YOU SEARCH: ros2 launch file run on boot linux
# SAID SOMETHING ABOUT GREPPING STUFF

export HOME=/home/pi
alias python="python3"
source /opt/ros/humble/setup.bash

cd /home/pi/SpaceGrantEngine
make
source /home/pi/SpaceGrantEngine/install/setup.bash
cd /home/pi/SpaceGrantEngine/launch
ros2 launch manual_control.launch.py
