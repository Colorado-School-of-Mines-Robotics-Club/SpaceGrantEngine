#!/usr/bin/env python3

import subprocess
import os
import sys

def install_apt_packages(packages):
    if not isinstance(packages, list):
        packages = [packages]
    subprocess.check_call(["sudo", "apt", "install", "-y"] + packages)


ros_distro = os.getenv("ROS_DISTRO")
if ros_distro is None:
    print("ROS_DISTRO is not set. Has ROS been sourced?")
    exit(1)

subprocess.check_call(["sudo", "apt", "update"])

# Install misic tools
install_apt_packages(["nano", "wget", "curl"])

# Install deps (system)
install_apt_packages(
    [
        f"ros-{ros_distro}-depthai",
        "python3-cv-bridge",
        "python3-pip",
        "usbutils",
        "udev"
    ]
)

subprocess.check_call(["bash", "-c", "echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"' | sudo tee /etc/udev/rules.d/80-movidius.rules"])

if "--apt_only" in sys.argv:
    exit()

def install_pip_packages(packages, check_return_code=True):
    if not isinstance(packages, list):
        packages = [packages]
    command = ["pip3", "install"] + packages
    if check_return_code:
        subprocess.check_call(command)
    else:
        subprocess.run(command)

subprocess.check_call(["pip3", "install", "pip", "--upgrade"])

install_pip_packages(
    [
        "black",
        "isort",
        "ruff",
        "pyserial",
        "numpy<1.25.0",
        "scipy",
        "depthai",
        "opencv-contrib-python==4.8.0.74",
        "oakutils",
    ]
)
install_pip_packages(["./extern/openVO", "./extern/linux-js"], check_return_code=False)

install_pip_packages("RPi.GPIO", check_return_code=False)
