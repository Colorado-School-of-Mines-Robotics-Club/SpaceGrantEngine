from setuptools import setup
from setuptools.command.easy_install import EasyInstallDeprecationWarning
from setuptools._deprecation_warning import SetuptoolsDeprecationWarning
import os
from glob import glob
import warnings
warnings.filterwarnings("ignore", category=EasyInstallDeprecationWarning)
warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)


package_name = "sgengine"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Colorado School of Mines - Robotics Club",
    maintainer_email="jcdavis@mines.edu",
    description="SpaceGrantEngine - ROS2 Utilities Package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pico            = sgengine.hardware.pico.pico_node:main",
            "steamcontroller = sgengine.hardware.controller.steamcontroller_node:main",
            "odometry        = sgengine.odometry.odometry_node:main",
            "pathfinding     = sgengine.pathfinding.pathfinding_node:main",
        ],
    },
)
