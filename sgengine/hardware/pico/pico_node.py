import rclpy

from std_msgs.msg import String
from ...abstract_node import AbstractNode
from .pico_comms import PicoComms


class PicoNode(AbstractNode, PicoComms):
    """Node for handling Raspberry Pi Pico"""

    def __init__(self) -> None:
        AbstractNode.__init__(self, "pico")
        PicoComms.__init__(self)

    def _main(self) -> None:
        def move_callback(msg: String) -> None:
            tokens = msg.data.split(" ")
            PicoComms.send_move_command(self, tokens[0], tokens[1])

        self.subscribe("pico/move_command", move_callback)
        pass


def main(args=None):
    """
    Main function which exclusively launches the Pico node
    """
    rclpy.init(args=args)
    pico = PicoNode()
    pico.main()
    # rclpy.spin(pico)
    # pico.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
