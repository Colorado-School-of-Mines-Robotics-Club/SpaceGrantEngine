# thanks stack overflow : https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python

import math
import threading
import time

import rclpy
from linux_js import XBOX_CONSTANTS, AxisEvent, Joystick
from rclpy.node import Node

from sgengine_messages.msg import XboxInput


class XboxControllerNode(Node):
    """Node for handling input from an Xbox Controller"""

    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self) -> None:
        # ros stuff
        Node.__init__(self, "xboxcontroller")
        self._publisher = self.create_publisher(
            XboxInput, "xbox_controller/all_inputs", 10
        )
        self._launched_auton = False

        self._publish_timer = self.create_timer(1.0 / 60.0, self.publish_inputs)

        # controller setup
        self._left_stick_y = 0.0
        self._right_stick_y = 0.0

        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=()
        )
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

        print("Running the xboxcontroller node")

    def publish_inputs(
        self,
    ):  # publish the buttons/triggers that you care about in this methode
        msg = XboxInput()
        msg.left_joystick_y = self._left_stick_y
        msg.right_joystick_y = self._right_stick_y
        self._publisher.publish(msg)

    def _monitor_controller(self):
        js = None
        while True:
            while True:
                try:
                    js = Joystick(0)
                    break
                except FileNotFoundError:
                    print("Controller not connected. Retrying in 5 seconds...")
                    time.sleep(5)

            print("Controller connected!")

            while True:
                event = None
                try:
                    event = js.poll()
                except OSError:
                    break
                if isinstance(event, AxisEvent):
                    if event.id == XBOX_CONSTANTS.L_STICK_Y:
                        self._left_stick_y = max(event.value / 32767, -1.0)
                    elif event.id == 4:
                        self._right_stick_y = max(event.value / 32767, -1.0)


def main(args=None):
    """
    Main function which exclusively launches the XboxController node
    """
    rclpy.init(args=args)
    controller = XboxControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
