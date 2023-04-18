import sys
import time
import subprocess
import rclpy
from rclpy.node import Node
from sgengine_messages.msg import TwoFloat

try:
    from steamcontroller import SteamController
except ModuleNotFoundError:
    print(
        "ERROR: Could not load steamcontroller library, node will not launch",
        file=sys.stderr,
    )


class SteamControllerNode(Node):
    """Node for handling input from SteamController"""

    move_max = 32767
    last_input_delay = -1
    input_delay = 0.05

    def __init__(self) -> None:
        Node.__init__(self, "steamcontroller")

        self._publisher = self.create_publisher(TwoFloat, "pico/move_command", 10)
        self._launched_auton = False

        def joystick(_, sci):
            if time.perf_counter() - self.last_input_delay < self.input_delay:
                return
            self.last_input_delay = time.perf_counter()

            if sci.buttons == 32768 and not self._launched_auton:
                self._launched_auton = True

                print("Launching auton_control")
                subprocess.run(
                    ["/home/pi/SpaceGrantEngine/scripts/launch_auton.sh"], check=False
                )
                print("auton_control exited...")

            x = sci.lpad_x
            y = sci.lpad_y

            x = x / self.move_max
            y = y / self.move_max

            x = max(min(1.0, x), -1.0)
            y = max(min(1.0, y), -1.0)

            msg = TwoFloat()
            msg.first = x
            msg.second = y
            self._publisher.publish(msg)

        self._sc = SteamController(callback=joystick)
        print("Running SteamControllerNode")
        self._sc.run()


def main(args=None):
    """
    Main function which exclusively launches the SteamController node
    """
    # If steamcontroller package not found, exit
    if "steamcontroller" not in sys.modules:
        return
    rclpy.init(args=args)
    controller = SteamControllerNode()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
