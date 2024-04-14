import logging

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from sgengine_messages.msg import MoveCommand

from .sg_logger import SG_Logger


class AutonomousNode(Node, SG_Logger):
    """Cam for handling depth-based obstancle avoidance"""

    def __init__(self) -> None:
        Node.__init__(self, "autonomous_node")
        SG_Logger.__init__(self)

        self._heading_subscription = self.create_subscription(
            Float32, "oak/simple_heading", self.process_heading, 10
        )
        self._pico_publisher = self.create_publisher(
            MoveCommand, "pico/move_command", 10
        )

        logging.info("Running Auto Control Node")

    def process_heading(self, msg: Float32) -> None:
        target_heading = msg.data
        logging.info(target_heading)
        target_speed = 0.25

        left_speed = max(min(target_speed + target_heading / 4.0, 5.0), 0.0)
        right_speed = max(min(target_speed - target_heading / 4.0, 5.0), 0.0)

        scaling_factor = 1.0 / max(left_speed, right_speed)

        left_speed = left_speed * scaling_factor
        right_speed = right_speed * scaling_factor

        move_command = MoveCommand()
        move_command.left = left_speed
        move_command.right = right_speed
        self._pico_publisher.publish(move_command)


def main(args=None):
    """
    Main function which exclusively launches the Manual Control node
    """
    rclpy.init(args=args)
    node = AutonomousNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
