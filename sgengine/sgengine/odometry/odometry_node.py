# pylint: skip-file

import sys
import rclpy
import atexit

from sgengine_messages.msg import RPYXYZ
from rclpy.node import Node
from sensor_msgs.msg import Image
from openVO import rot2RPY
from openVO.oakd import OAK_Camera, OAK_Odometer
import numpy as np
from cv_bridge import CvBridge


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

        self._pose_publisher = self.create_publisher(RPYXYZ, "/odom/rpy_xyz", 10)
        self._depth_publisher = self.create_publisher(Image, "/odom/depth", 10)
        self._rgb_publisher = self.create_publisher(Image, "/odom/rgb", 10)

        self._pose = None

        self._stopped = False

        # atexit.register(self._stop)

        self._bridge = CvBridge()

        self._run()

    def _stop(self) -> None:
        self._stopped = True
        self._cam.stop()

    def _run(self) -> None:
        print("Running the odometry node")
        while not self._stopped:
            self._odom.update()
            self._pose = self._odom.current_pose()

            pose = RPYXYZ()
            roll, pitch, yaw = rot2RPY(self._pose)
            x, y, z = (
                float(self._pose[0, 3]),
                float(self._pose[1, 3]),
                float(self._pose[2, 3]),
            )
            
            rep1, rep2 = [np.linalg.norm([roll[i], pitch[i], yaw[i]]) for i in [0, 1]]
            if rep1 > rep2:
                r = roll[1]
                p = pitch[1]
                y = yaw[1]
            else:
                r = roll[0]
                p = pitch[0]
                y = yaw[0]

            pose.roll = float(r)
            pose.pitch = float(p)
            pose.yaw = float(y)
            pose.x = float(x)
            pose.y = float(y)
            pose.z = float(z)

            depth_img = self._bridge.cv2_to_imgmsg(self._cam.depth)
            rgb_img = self._bridge.cv2_to_imgmsg(self._cam.rgb)
            self._pose_publisher.publish(pose)
            self._depth_publisher.publish(depth_img)
            self._rgb_publisher.publish(rgb_img)
        self._cam.stop()

def main(args=None):
    """
    Main function which exclusively launches the Odometer node
    """
    rclpy.init(args=args)
    odometer = OdometryNode()
    rclpy.spin(odometer)
    odometer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
