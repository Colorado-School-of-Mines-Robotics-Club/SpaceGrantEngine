import sys

import rclpy
from rclpy.node import Node
from ros2node.api import get_node_names

from sgengine_messages.msg import TwoFloat

from .pico_comms import PicoComms


class PicoNode(Node, PicoComms):
    """Node for handling Raspberry Pi Pico"""

    def __init__(self) -> None:
        Node.__init__(self, "pico")
        PicoComms.__init__(self)

        def move_callback(msg: TwoFloat) -> None:
            PicoComms.send_move_command(
                self, int(msg.first * 255), int(msg.second * 255)
            )

        self.subscription = self.create_subscription(
            TwoFloat, "pico/move_command", move_callback, 10
        )

        print("Running PicoNode")


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
        print(
            "Pico node is not launching because the manual_pico node is already running!",
            file=sys.stderr,
        )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
