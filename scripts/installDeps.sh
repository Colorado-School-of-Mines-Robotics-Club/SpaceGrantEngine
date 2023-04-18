#!/bin/bash

# Any subsequent commands which fail will cause the shell script to exit immediately
set -e

# Python w/ pip:
apt-get install -y python3-pip

# Extra dev tools:
apt-get install nano wget curl -y

# CI tools
apt install -y python3-pylint-common python3-flake8 black

# Engine deps
# Prefer apt packages when possible
apt install -y python3-numpy python3-opencv ros-humble-depthai python3-flask python3-scipy python3-cv-bridge python3-open3d
# Otherwise, use pip
pip3 install depthai

# Install deps from extern
apt install -y python3-libusb1
pip3 install /root/extern/openVO
pip3 install /root/extern/steamcontroller
