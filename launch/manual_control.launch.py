from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sgengine',
            executable='manual',
        ),
        Node(
            package='sgengine',
            executable='pico',
        ),
        Node(
            package='sgengine',
            executable='xboxcontroller',
        )
    ])
