# pylint: skip-file

import time
import rclpy

# from std_msgs.msg import String
from sgengine_messages.msg import TwoFloat
from rclpy.node import Node
from steamcontroller import SteamController


class SteamControllerNode(Node):
    """Node for handling input from SteamController"""

    MAX = 32767
    LAST_PACKET = -1
    DELAY = 0.25

    def __init__(self) -> None:
        Node.__init__(self, "steamcontroller")

        self._publisher = self.create_publisher(TwoFloat, "pico/move_command", 10)

        def joystick(_, sci):
            if time.perf_counter() - self.LAST_PACKET < self.DELAY:
                return
            self.LAST_PACKET = time.perf_counter()

            x = sci.lpad_x
            y = sci.lpad_y

            x = x / self.MAX
            y = y / self.MAX

            x = max(min(1.0, x), -1.0)
            y = max(min(1.0, y), -1.0)

            msg = TwoFloat()
            msg.first = x
            msg.second = y
            self._publisher.publish(msg)

        self._sc = SteamController(callback=joystick)
        print("Running steamcontroller")
        self._sc.run()


def main(args=None):
    """
    Main function which exclusively launches the SteamController node
    """
    rclpy.init(args=args)
    controller = SteamControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
