import logging
import sys

import time
import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names

from sgengine_messages.msg import MoveCommand

from ...sg_logger import SG_Logger
from .pico_comms import PicoComms


class PicoNode(Node, PicoComms, SG_Logger):
    """Node for handling Raspberry Pi Pico"""

    def __init__(self) -> None:
        Node.__init__(self, "pico")
        PicoComms.__init__(self)
        SG_Logger.__init__(self)

        # timer for keeping track of pico communication delays and preventing runaways
        timer_period = 1  # seconds
        self.last_time = (
            time.time()  # variable to keep track of time since the last move_callback call
        )
        self.timer = self.create_timer(timer_period, self.timer_callback)

        def move_callback(msg: MoveCommand) -> None:
            PicoComms.send_move_command(self, int(msg.left * 255), int(msg.right * 255))

            # save time of when the callback is called
            self.last_time = time.time()

        self.subscription = self.create_subscription(
            MoveCommand, "pico/move_command", move_callback, 10
        )

        logging.info("Running PicoNode")

    def timer_callback(self) -> None:
        # compare time of last move command with current time
        # if its greater than one second, send a move command of 0 speed for all motors
        if self.last_time - time.time() > 1.0:
            PicoComms.send_move_command(self, 0, 0)
            logging.error("No command recieved for a second. Stopping bot movement.")


def main(args=None):
    """
    Main function which exclusively launches the Pico node if one isn't running already
    """
    rclpy.init(args=args)
    should_launch = True
    if sys.argv.count("--no_duplicates") > 0:
        list_node = rclpy.create_node("list_node")
        available_nodes = get_node_names(node=list_node, include_hidden_nodes=False)
        for name, _, _ in available_nodes:
            if name == "manual_pico":
                should_launch = False
        list_node.destroy_node()

    if should_launch:
        pico = PicoNode()
        rclpy.spin(pico)
        pico.destroy_node()
    else:
        logging.warning(
            "Pico node is not launching because the manual_pico node is already running!"
        )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
