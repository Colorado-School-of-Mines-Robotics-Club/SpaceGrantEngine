#!/bin/bash

set -e

# NEED THE ENVIRONMENT VARIABLES TO GET THE RIGHT INFO
# SEE THE TOP LINK IF YOU SEARCH: ros2 launch file run on boot linux
# SAID SOMETHING ABOUT GREPPING STUFF

export HOME=/home/pi
export WORKSPACE=$HOME/SpaceGrantEngine
alias python="python3"
source /opt/ros/humble/setup.bash

cd $WORKSPACE
make
source $WORKSPACE/install/setup.bash
ros2 launch sgengine manual_control.launch.py
