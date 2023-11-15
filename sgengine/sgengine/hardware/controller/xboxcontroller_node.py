# thanks stack overflow : https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python

import math
import threading
import time

import subprocess
import rclpy
from linux_js import XBOX_CONSTANTS, AxisEvent, Joystick
from rclpy.node import Node

from sgengine_messages.msg import TwoFloat


class XboxControllerNode(Node):
    """Node for handling input from an Xbox Controller"""

    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self) -> None:
        # ros stuff
        Node.__init__(self, "xboxcontroller")
        self._publisher = self.create_publisher(TwoFloat, "pico/move_command", 10)
        self._launched_auton = False

        self._publish_timer = self.create_timer(1.0 / 60.0, self.publish_inputs)

        # controller setup
        self._target_linear = 0.0
        self._target_angular = 0.0

        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=()
        )
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

        print("Running the xboxcontroller node")

    def publish_inputs(
        self,
    ):  # if autonomous mode is not running, send the message to move the robot
        if self._launched_auton:
            return
        else:
            msg = TwoFloat()
            msg.first = self._target_angular
            msg.second = self._target_linear
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
                    if event.id == XBOX_CONSTANTS.L_STICK_X:
                        self._target_angular = min(
                            max(event.value / XBOX_CONSTANTS.MAX_AXIS_VALUE, -1.0), 1.0
                        )
                    elif event.id == XBOX_CONSTANTS.L_STICK_Y:
                        self._target_linear = min(
                            max(event.value / XBOX_CONSTANTS.MAX_AXIS_VALUE, -1.0), 1.0
                        )
                # button presses would prolly be an elif here
                elif isinstance(event, ButtonEvent):
                    if event.id == #buttton:
                        if not self._launched_auton:
                            #fill command list in later with real stuff
                            command_list = ["echo"]
                            self._auton_process = subprocess.Popen(command_list)
                            self._launched_auton = True
                    if event.id == #other button:
                        if self._launched_auton:
                            self._auton_process.terminate()
                            self._launched_auton = False


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