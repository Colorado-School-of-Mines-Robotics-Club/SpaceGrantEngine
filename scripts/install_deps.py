#!/bin/python3

import subprocess
import os

def install_pip_packages(packages):
    if type(packages) != list:
        packages = [packages]
    subprocess.check_call(["sudo", "pip3", "install"] + packages + ["--upgrade"])

def install_apt_packages(packages):
    if type(packages) != list:
        packages = [packages]
    subprocess.check_call(["sudo", "apt", "install", "-y"] + packages)


ros_distro = os.getenv("ROS_DISTRO")
if ros_distro is None:
    print("ROS_DISTRO is not set. Has ROS been sourced?")
    exit(1)

subprocess.check_call(["sudo", "apt", "update"])

misic_tools = [
    "nano",
    "wget",
    "curl"
]
install_apt_packages(misic_tools)

engine_deps = [
    f"ros-{ros_distro}-depthai",
    "python3-cv-bridge",
]
install_apt_packages(engine_deps)

subprocess.check_call(["sudo", "wget", "https://bootstrap.pypa.io/get-pip.py", "-P", "/root/"])
subprocess.check_call(["sudo", "python3", "/root/get-pip.py"])

pip_packages = [
    "pylint",
    "flake8",
    "black",
    "numpy",
    "scipy",
    "depthai",
    "opencv-contrib-python==4.8.0.74",
    "./extern/openVO",
    "./extern/steamcontroller",
    "./extern/oakutils"
]
install_pip_packages(pip_packages)