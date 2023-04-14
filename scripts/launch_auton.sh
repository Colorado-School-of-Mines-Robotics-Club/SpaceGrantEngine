#!/bin/bash

export HOME=/home/pi
alias python="python3"
source /opt/ros/humble/setup.bash

source /home/pi/SpaceGrantEngine/install/setup.bash
cd /home/pi/SpaceGrantEngine/launch
ros2 launch auton_control.launch.py

