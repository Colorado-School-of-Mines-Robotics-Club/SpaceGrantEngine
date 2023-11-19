#!/usr/bin/env python3

import os
import subprocess

if os.geteuid() != 0:
    exit("This script must be run as root!")


def install_apt_packages(packages):
    if packages is None:
        return
    if type(packages) is tuple:
        packages = list(packages)
    if not isinstance(packages, list):
        packages = [packages]
    subprocess.check_call(["apt", "install", "-y"] + packages)


def install_pip_packages(packages):
    if packages is None:
        return
    if type(packages) is tuple:
        packages = list(packages)
    if not isinstance(packages, list):
        packages = [packages]
    subprocess.check_call(["pip3", "install"] + packages)


def run_bash_command(command):
    return subprocess.check_call(["bash", "-c", command])


run_bash_command('echo "" > /etc/apt/apt.conf.d/99needrestart')

subprocess.check_call(["apt", "update"])

install_apt_packages("python3-pip")
install_pip_packages("simple-term-menu")

from simple_term_menu import TerminalMenu  # noqa: E402


def create_selection_menu(entries, title=None):
    return TerminalMenu(
        title=title,
        menu_entries=entries,
        preselected_entries=range(len(entries)),
        multi_select=True,
        show_multi_select_hint=True,
        clear_screen=True,
        raise_error_on_interrupt=True,
        multi_select_select_on_accept=False,
        multi_select_empty_ok=True,
    )


helpful_tools = None


terminal_menu = create_selection_menu(
    title="Select Install Options",
    entries=[
        "Run full upgrade",
        "Install extra tools",
        "Install raspi-config",
        "Configure ROS2",
        "Remove unnecessary desktop packages",
        "Configure device comms",
        "Disable scan after apt install",
        "Disable unattended-upgrades service",
    ],
)
terminal_menu.show()
install_steps = terminal_menu.chosen_menu_entries


def step_selected(step):
    return install_steps is not None and step in install_steps


if step_selected("Disable unattended-upgrades service"):
    subprocess.check_call(["systemctl", "stop", "unattended-upgrades.service"])
    subprocess.check_call(["systemctl", "disable", "unattended-upgrades.service"])

if step_selected("Remove unnecessary desktop packages"):
    subprocess.run(["apt", "remove", "-y", "--purge", "thunderbird", "libreoffice*"])

if step_selected("Install extra tools"):
    terminal_menu = create_selection_menu(
        title="Helpful tools to install...",
        entries=[
            "htop",
            "net-tools",
            "wget",
            "curl",
            "git",
            "gcc",
            "g++",
            "make",
            "perl",
        ],
    )
    terminal_menu.show()
    helpful_tools = terminal_menu.chosen_menu_entries

if step_selected("Run full upgrade"):
    subprocess.check_call(["apt", "upgrade", "-y"])
    subprocess.check_call(["snap", "refresh"])

if step_selected("Configure device comms"):
    try:
        f = open("/boot/firmware/cmdline.txt", "r")
        tokens = f.read().split()
        f.close()
        if "console=serial0,115200" in tokens:
            tokens.remove("console=serial0,115200")
        f = open("/boot/firmware/cmdline.txt", "w")
        f.write(" ".join(tokens))
        f.close()
    except:  # noqa: E722
        pass

    subprocess.check_call(
        [
            "bash",
            "-c",
            'echo \'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"\' | tee /etc/udev/rules.d/80-movidius.rules',
        ]
    )
    subprocess.check_call(
        ["bash", "-c", "udevadm control --reload-rules && udevadm trigger"]
    )
subprocess.check_call(["apt", "autoremove", "-y"])

install_apt_packages(helpful_tools)

if step_selected("Install raspi-config"):
    install_apt_packages("raspi-config")

if step_selected("Configure ROS2"):
    install_apt_packages("software-properties-common")
    subprocess.check_call(["add-apt-repository", "-y", "universe"])
    run_bash_command(
        "curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
    )
    run_bash_command(
        'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'
    )
    subprocess.check_call(["apt", "update"])

    ros_version = "humble"
    install_apt_packages([f"ros-{ros_version}-ros-base", "python3-colcon-*"])


print("Done!")
