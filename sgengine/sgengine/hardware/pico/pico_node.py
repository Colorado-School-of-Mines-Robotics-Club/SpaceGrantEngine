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

        self._last_time = time.time()

        def move_callback(msg: MoveCommand) -> None:
            self._last_time = time.time()
            leftval, rightval = int(msg.left * 255), int(msg.right * 255)
            leftval = max(-254, min(leftval, 255))
            rightval = max(-254, min(rightval, 255))
            self.send_move_command(leftval, rightval)

        self._subscription = self.create_subscription(
            MoveCommand, "pico/move_command", move_callback, 10
        )

        logging.info("Running PicoNode")


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
