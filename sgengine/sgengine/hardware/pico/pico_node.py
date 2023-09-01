import rclpy


from rclpy.node import Node

from sgengine_messages.msg import TwoFloat
from .pico_comms import PicoComms


class PicoNode(Node, PicoComms):
    """Node for handling Raspberry Pi Pico"""

    def __init__(self) -> None:
        Node.__init__(self, "pico")
        PicoComms.__init__(self)

        def move_callback(msg: TwoFloat) -> None:
            PicoComms.send_move_command(self, msg.first, msg.second)

        self.subscription = self.create_subscription(
            TwoFloat, "pico/move_command", move_callback, 10
        )

        print("Running PicoNode")


def main(args=None):
    """
    Main function which exclusively launches the Pico node
    """
    rclpy.init(args=args)
    pico = PicoNode()
    rclpy.spin(pico)
    pico.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
