from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sgengine',
            executable='pico',
            name='pico'
        ),
        Node(
            package='sgengine',
            executable='odometry',
            name='odometry'
        ),
        Node(
            package='sgengine',
            executable='pathfinding',
            name='pathfinding'
        )
    ])
    