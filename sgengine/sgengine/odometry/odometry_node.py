# pylint: skip-file

import sys
import rclpy
import atexit

from sgengine_messages.msg import RPYXYZ
from rclpy.node import Node
from sensor_msgs.msg import Image
from openVO import rot2RPY
from openVO import OAK_Camera, OAK_Odometer


class OdometryNode(Node):
    """Node for running visual odometry"""

    def __init__(self) -> None:
        Node.__init__(self, "odometer")

        self._cam = OAK_Camera(
            median_filter=7,
            stereo_threshold_filter_min_range=200,
            stereo_threshold_filter_max_range=20000,
        )
        self._odom = OAK_Odometer(
            self._cam,
            nfeatures=250,
        )
        self._cam.start(block=True)

        self._pose_publisher = self.create_publisher(RPYXYZ, "odom/rpy_xyz", 10)
        self._depth_publisher = self.create_publisher(Image, "odom/depth", 10)
        self._rgb_publisher = self.create_publisher(Image, "odom/rgb", 10)

        self._pose = None

        self._stopped = False

        atexit.register(self._stop)

        self._run()

    def _stop(self) -> None:
        self._stopped = True
        self._cam.stop()

    def _run(self) -> None:
        while not self._stopped:
            self._odom.update()
            self._pose = self._odom.current_pose()

            pose = RPYXYZ()
            r, p, y = rot2RPY(self._pose)
            x, y, z = (
                float(self._pose[0, 3]),
                float(self._pose[1, 3]),
                float(self._pose[2, 3]),
            )

            pose.roll = r
            pose.pitch = p
            pose.yaw = y
            pose.x = x
            pose.y = y
            pose.z = z

            self._pose_publisher.publish(pose)
            self._depth_publisher.publish(self._cam.depth)
            self._rgb_publisher.publish(self._cam.rgb)
        self._cam.stop()


def main(args=None):
    """
    Main function which exclusively launches the Odometer node
    """
    if "steamcontroller" not in sys.modules:
        return
    rclpy.init(args=args)
    odometer = OdometryNode()
    rclpy.spin(odometer)
    odometer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
