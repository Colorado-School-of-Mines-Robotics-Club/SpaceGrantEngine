from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sgengine',
            namespace='gui_node',
            executable='gui_node:main',
            name='gui_node'
        ),
    ])
    