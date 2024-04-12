from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="sgengine",
                executable="manual",
                name="manual",
                respawn=True,
                respawn_delay=1,
            ),
            Node(
                package="sgengine",
                executable="pico",
                name="manual_pico",
                respawn=True,
                respawn_delay=5,
            ),
            Node(
                package="sgengine",
                executable="xboxcontroller",
                name="manual_xboxcontroller",
                respawn=True,
                respawn_delay=5,
            ),
        ]
    )
