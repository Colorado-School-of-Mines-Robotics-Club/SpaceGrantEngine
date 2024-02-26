import cv2
import numpy as np
import rclpy
import time
from cv_bridge import CvBridge
from oakutils import ApiCamera, set_log_level
from oakutils.nodes import (create_color_camera, create_neural_network,
                            create_stereo_depth, create_xout, get_nn_frame)
from openVO import rot2RPY
from openVO.oakd import OAK_Odometer
from rclpy.node import Node
from sensor_msgs.msg import Image

from sgengine_messages.msg import RPYXYZ


class OdometryCam:
    def __init__(self):
        self._cam = ApiCamera(
            color_size=(1920, 1080),
            mono_size=(640, 400),
            primary_mono_left=True,
        )
        self._calibration = self._cam.calibration
        self._Q = self._calibration.stereo.Q_cv2

        color = create_color_camera(self._cam.pipeline)
        stereo, left, right = create_stereo_depth(self._cam.pipeline)
        nn = create_neural_network(
            self._cam.pipeline,
            input_link=stereo.depth,
            blob_path="depth_proc.blob",
        )

        nn_xout = create_xout(self._cam.pipeline, nn.out, "nn_depth")
        color_xout = create_xout(self._cam.pipeline, color.preview, "color")
        disparity_xout = create_xout(self._cam.pipeline, stereo.disparity, "disparity")
        left_xout = create_xout(self._cam.pipeline, stereo.rectifiedLeft, "left")

        self._rgb = np.zeros((1080, 1920, 3))
        self._cam.add_callback("color", self._update_color)
        self._depth = np.zeros((400, 640, 1))
        self._cam.add_callback("nn_depth", self._update_depth)

        self._disparity = np.zeros((400, 640, 1))
        self._im3d = np.zeros((400, 640, 3))
        self._left = np.zeros((400, 640, 1))
        self._cam.add_callback("disparity", self._update_disparity)
        self._cam.add_callback("left", self._update_left)

        self._nodes = [
            color,
            stereo,
            left,
            right,
            nn_xout,
            color_xout,
            disparity_xout,
            left_xout,
        ]

    def _update_left(self, frame):
        self._left = frame.getCvFrame()

    def _update_disparity(self, frame):
        self._disparity = frame.getCvFrame()
        self._im3d = cv2.reprojectImageTo3D(self._disparity, self._Q)

    def _update_color(self, frame):
        self._rgb = frame.getCvFrame()

    def _update_depth(self, frame):
        self._depth = get_nn_frame(frame, channels=1, frame_size=(640, 400))

    def _crop_to_valid_primary_region(self, img: np.ndarray) -> np.ndarray:
        if self._calibration.primary is None:
            err_msg = "Primary calibration is not available."
            raise RuntimeError(err_msg)
        if self._calibration.primary.valid_region is None:
            err_msg = "Primary valid region is not available."
            raise RuntimeError(err_msg)
        return img[
            self._calibration.primary.valid_region[
                1
            ] : self._calibration.primary.valid_region[3],
            self._calibration.primary.valid_region[
                0
            ] : self._calibration.primary.valid_region[2],
        ]

    def compute_im3d(self):
        im3d = self._crop_to_valid_primary_region(self._im3d)
        disparity = self._crop_to_valid_primary_region(self._disparity)
        left = self._crop_to_valid_primary_region(self._left)
        print(im3d.shape, disparity.shape, left.shape)
        return im3d, disparity.astype(np.uint8), left.astype(np.uint8)

    def start(self, block):
        self._cam.start(blocking=False)

    def stop(self):
        self._cam.stop()

    @property
    def depth(self):
        return self._depth

    @property
    def rgb(self):
        return self._rgb


class OdometryNode(Node):
    """Node for running visual odometry"""

    def __init__(self) -> None:
        Node.__init__(self, "odometer")

        self._cam = OdometryCam()
        self._odom = OAK_Odometer(
            self._cam,
            nfeatures=500,
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

            print(f"Pose: r: {r}, p: {p}, y: {y}, x: {x}, y: {y}, z: {z}")

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
    set_log_level("DEBUG")
    rclpy.init(args=args)
    odometer = OdometryNode()
    rclpy.spin(odometer)
    odometer.destroy_node()
    rclpy.shutdown()
