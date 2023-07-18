#!/bin/bash

# Any subsequent commands which fail will cause the shell script to exit immediately
set -e

# If user is not superuser, elevate with sudo
if [ "$EUID" != 0 ]; then
    sudo "$0" "$@"
    exit $?
fi

# If ROS environment is not configured, stop
if [ -z "$ROS_DISTRO" ]
then
    echo "ROS_DISTRO is not set. Have you sourced ROS?"
    exit 1
fi

# Misic dev tools:
apt-get install -y nano wget curl

# Python:
apt-get install -y python3
# Pip:
wget https://bootstrap.pypa.io/get-pip.py -P /root/ && python3 /root/get-pip.py

# CI tools
apt install -y python3-pylint-common python3-flake8 black

# Engine deps
# Prefer apt packages when possible
apt install -y python3-numpy python3-opencv ros-$ROS_DISTRO-depthai python3-flask python3-scipy python3-cv-bridge python3-libusb1

# Otherwise, use pip    
pip3 install depthai ./extern/openVO ./extern/steamcontroller ./extern/oakutils
