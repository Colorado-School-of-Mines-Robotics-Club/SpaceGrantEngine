#!/bin/bash

alias python="python3"
source /opt/ros/humble/setup.bash

cd /home/pi/SpaceGrantEngine
make
source /home/pi/SpaceGrantEngine/install/setup.bash
cd /home/pi/SpaceGrantEngine/launch
ros2 launch manual_control.launch.py
