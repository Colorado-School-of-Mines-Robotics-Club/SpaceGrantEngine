#!/usr/bin/env bash

# setup
sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-ros/main/install_dependencies.sh | sudo bash
sudo apt install libopencv-dev
sudo apt install python3-rosdep
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/humble/setup.bash
./src/depthai-ros/build.sh

source install/setup.bash
