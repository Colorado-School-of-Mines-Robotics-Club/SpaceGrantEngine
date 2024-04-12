import logging

import rclpy
from rclpy.node import Node

from sgengine_messages.msg import MoveCommand, XboxInput

from .sg_logger import SG_Logger


class ManualNode(Node, SG_Logger):
    """Node for handling manual robot control"""

    def __init__(self) -> None:
        Node.__init__(self, "manual_control")
        SG_Logger.__init__(self)

        self.publisher = self.create_publisher(MoveCommand, "pico/move_command", 10)

        def input_callback(xbox_input: XboxInput) -> None:
            pico_command = MoveCommand()
            pico_command.left = max(-1.0, min(1.0, -xbox_input.left_joystick_y))
            pico_command.right = max(-1.0, min(1.0, -xbox_input.right_joystick_y))
            self.publisher.publish(pico_command)

        self.subscription = self.create_subscription(
            XboxInput, "xbox_controller/all_inputs", input_callback, 10
        )

        logging.info("Running Manual Control Node")


def main(args=None):
    """
    Main function which exclusively launches the Manual Control node
    """
    rclpy.init(args=args)
    manual = ManualNode()
    rclpy.spin(manual)
    manual.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
