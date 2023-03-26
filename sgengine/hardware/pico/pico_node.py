import rclpy

from ...abstract_node import AbstractNode
from .pico_comms import PicoComms


class PicoNode(AbstractNode, PicoComms):
    """Node for handling Raspberry Pi Pico"""

    def __init__(self) -> None:
        AbstractNode.__init__(self, "pico")
        PicoComms.__init__(self)

    def _main(self) -> None:
        def move_callback(msg: str) -> None:
            """
            Message must be in format 'speed direction'
            """
            tokens = msg.split(" ")
            PicoComms.send_instruction(self, tokens[0], tokens[1])

        self.subscribe("pico move", move_callback)
        pass


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
