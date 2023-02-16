from setuptools import setup
import os
from glob import glob


package_name = "sgengine"

package_dirs = [package_name]
for root, dirs, files in os.walk(package_name):
    for directory in dirs:
        if directory == "__pycache__":
            continue
        package_dirs.append(os.path.join(root, directory))
package_dirs = [pack.replace("//", ".") for pack in package_dirs if "__pycache__ not in pack"]
package_dirs = [pack.replace("/", ".") for pack in package_dirs if "__pycache__ not in pack"]
package_dirs = [pack.replace("\\", ".") for pack in package_dirs if "__pycache__ not in pack"]
packages = [*package_dirs]

setup(
    name=package_name,
    version="0.0.1",
    packages=packages,
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
            "test = sgengine.test_node:main",
        ],
    },
)
