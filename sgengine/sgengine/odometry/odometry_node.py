import logging
import pickle
import time
from pathlib import Path
from threading import Thread

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from oakutils import set_log_level
from oakutils.calibration import CalibrationData
from rclpy.node import Node
from sensor_msgs.msg import Image

from sgengine_messages.msg import RPYXYZ

from ..sg_logger import SG_Logger
from .oak_odometer import OAK_Odometer
from .rot2RPY import rot2RPY


class OdometryNode(Node, SG_Logger):
    """Node for running visual odometry"""

    def __init__(self) -> None:
        Node.__init__(self, "odometer")
        SG_Logger.__init__(self)

        self._odom = OAK_Odometer(
            nfeatures=500,
        )

        self._calibration: CalibrationData = None
        self._left = None
        self._disparity = None
        self._im3d = None

        calibration_file = Path("tmp") / "calibration.pkl"
        while True:
            if calibration_file.exists():
                break
            logging.warning("Calibration file not found, waiting for calibration data")
            time.sleep(0.5)
        with calibration_file.open("rb") as f:
            self._calibration = pickle.load(f)

        self._left_subscription = self.create_subscription(
            Image, "oak/left_image", self._update_left, 10
        )
        self._disparity_subscription = self.create_subscription(
            Image, "oak/disparity_image", self._update_disparity, 10
        )

        self._pose = None
        self._stopped = False
        self._bridge = CvBridge()
        self._pose_publisher = self.create_publisher(RPYXYZ, "/odom/rpy_xyz", 10)

        logging.info("Running the odometry node")
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def _update_calibration(self, calibration):
        self._calibration = calibration

    def _update_left(self, frame: Image):
        self._left = self._bridge.imgmsg_to_cv2(frame)

    def _update_disparity(self, frame: Image):
        self._disparity = self._bridge.imgmsg_to_cv2(frame)
        self._im3d = cv2.reprojectImageTo3D(
            self._disparity, self._calibration.stereo.Q_cv2
        )

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

    def _compute_im3d(self):
        im3d = self._crop_to_valid_primary_region(self._im3d)
        disparity = self._crop_to_valid_primary_region(self._disparity)
        left = self._crop_to_valid_primary_region(self._left)
        return im3d, disparity.astype(np.uint8), left.astype(np.uint8)

    def _run(self) -> None:
        while not self._stopped:
            if (
                self._left is None
                or self._disparity is None
                or self._im3d is None
                or self._calibration is None
            ):
                logging.info("Waiting for:")
                if self._left is None:
                    logging.info("Left")
                if self._disparity is None:
                    logging.info("Disparity")
                if self._im3d is None:
                    logging.info("Im3d")
                if self._calibration is None:
                    logging.info("Calibration")
                logging.info("")
                time.sleep(0.5)
                continue
            im3d, disparity, rect = self._compute_im3d()
            self._odom.update(im3d, disparity, rect)
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

            logging.info(f"Pose: r: {r}, p: {p}, y: {y}, x: {x}, y: {y}, z: {z}")

            self._pose_publisher.publish(pose)

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
