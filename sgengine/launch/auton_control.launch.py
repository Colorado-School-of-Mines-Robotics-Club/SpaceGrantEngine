from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sgengine',
            executable='pico',
            name='auton_pico',
            arguments=["--no_duplicates"]
        ),
        Node(
            package='sgengine',
            executable='odometry',
            name='auton_odometry'
        ),
        Node(
            package='sgengine',
            executable='pathfinding',
            name='auton_pathfinding'
        )
    ])
    