# thanks stack overflow : https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python

import logging
import math
import os
import subprocess
import time
from threading import Thread

import rclpy
from linux_joystick import XBOX_CONSTANTS, AxisEvent, ButtonEvent, Joystick
from rclpy.node import Node

from sgengine_messages.msg import XboxInput

from ...sg_logger import SG_Logger


class ThreadWithException(Thread):
    def run(self):
        try:
            super().run()
        except Exception as e:
            print(f"Exception in thread {self.name}: {e}")
            os._exit(1)


class XboxControllerNode(Node, SG_Logger):
    """Node for handling input from an Xbox Controller"""

    MAX_JOY_VAL = math.pow(2, 15)

    def __init__(self) -> None:
        # ros stuff
        Node.__init__(self, "xboxcontroller")
        SG_Logger.__init__(self)

        self._publisher = self.create_publisher(
            XboxInput, "xbox_controller/all_inputs", 10
        )
        self._launched_auton = False

        self._publish_timer = self.create_timer(1.0 / 60.0, self.publish_inputs)

        # controller setup
        self._left_stick_y = 0.0
        self._right_stick_y = 0.0

        self._monitor_thread = ThreadWithException(
            target=self._monitor_controller, daemon=True
        )
        self._monitor_thread.start()

        logging.info("Running the xboxcontroller node")

    def publish_inputs(
        self,
    ):  # if autonomous mode is not running, send the message to move the robot
        if self._launched_auton:
            return
        else:
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
                    logging.warning(
                        "Controller not connected. Retrying in 5 seconds..."
                    )
                    time.sleep(5)

            logging.info("Controller connected!")

            while True:
                event = None
                try:
                    event = js.poll()
                except OSError:
                    break
                if isinstance(event, AxisEvent):
                    if event.id == XBOX_CONSTANTS.L_STICK_Y_ID:
                        self._left_stick_y = max(
                            event.value / AxisEvent.MAX_AXIS_VALUE, -1.0
                        )
                    elif event.id == XBOX_CONSTANTS.R_STICK_Y_ID:
                        self._right_stick_y = max(
                            event.value / AxisEvent.MAX_AXIS_VALUE, -1.0
                        )
                # Only handle button presses on press down
                elif isinstance(event, ButtonEvent) and event.value is True:
                    if event.id == XBOX_CONSTANTS.START_BUTTON_ID:
                        if not self._launched_auton:
                            logging.info("Auton start received")
                            command_list = [
                                "bash",
                                "-c",
                                "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch launch/auton_control.launch.py",
                            ]
                            self._auton_process = subprocess.Popen(command_list)
                            self._launched_auton = True
                        else:
                            logging.info(
                                "Auton start received, but auton already launched!"
                            )
                    if event.id == XBOX_CONSTANTS.BACK_BUTTON_ID:
                        if self._launched_auton:
                            logging.info("Auton stop received")
                            self._auton_process.terminate()
                            self._launched_auton = False
                        else:
                            logging.warning(
                                "Auton stop received, but auton not launched!"
                            )


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
