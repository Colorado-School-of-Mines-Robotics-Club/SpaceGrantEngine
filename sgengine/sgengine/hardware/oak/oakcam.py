import logging
import pickle
import time
from collections import deque
from pathlib import Path

import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from oakutils import ApiCamera
from oakutils.nodes import (
    create_neural_network,
    create_stereo_depth,
    create_xout,
    get_nn_data,
)
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from ...sg_logger import SG_Logger


class OakCam(Node, SG_Logger):
    """Cam for handling depth-based obstancle avoidance"""

    def __init__(self) -> None:
        Node.__init__(self, "oak")
        SG_Logger.__init__(self)
        self._cam = ApiCamera(
            color_size=(1920, 1080),
            mono_size=(640, 400),
            primary_mono_left=True,
        )
        self._calibration = self._cam.calibration

        # write the calibration data to a file (MUST NOT CONTAIN OPEN3D OBJECTS)
        calibration_file = Path("tmp") / "calibration.pkl"
        Path("tmp").mkdir(parents=True, exist_ok=True)
        with calibration_file.open("wb") as f:
            pickle.dump(self._calibration, f, pickle.HIGHEST_PROTOCOL)

        self._buffer: deque[int] = deque(maxlen=10)

        self._bridge = CvBridge()

        depth, left, right = create_stereo_depth(self._cam.pipeline, fps=25)
        nn = create_neural_network(
            self._cam.pipeline, depth.depth, Path("data") / "simplePathfinding.blob"
        )

        # create xout links for the data streams we want to publish
        xout_nn = create_xout(self._cam.pipeline, nn.out, "nn")
        xout_rectleft = create_xout(self._cam.pipeline, depth.rectifiedLeft, "rectleft")
        xout_rectright = create_xout(
            self._cam.pipeline, depth.rectifiedRight, "rectright"
        )
        xout_depth = create_xout(self._cam.pipeline, depth.depth, "depth")
        xout_disparity = create_xout(self._cam.pipeline, depth.disparity, "disparity")

        # save the nodes
        self._nodes = [
            depth,
            left,
            right,
            nn,
            xout_nn,
            xout_rectleft,
            xout_rectright,
            xout_depth,
            xout_disparity,
        ]

        # create any output publishers
        self._publisher = self.create_publisher(Float32, "oak/simple_heading", 10)
        self._leftpub = self.create_publisher(Image, "oak/left_image", 10)
        self._rightpub = self.create_publisher(Image, "oak/right_image", 10)
        self._depthpub = self.create_publisher(Image, "oak/depth_image", 10)
        self._disparitypub = self.create_publisher(Image, "oak/disparity_image", 10)

        # add the callbacks
        self._cam.add_callback("nn", self._nn_callback)
        self._cam.add_callback("rectleft", self._left_callback)
        self._cam.add_callback("rectright", self._right_callback)
        self._cam.add_callback("depth", self._depth_callback)
        self._cam.add_callback("disparity", self._disparity_callback)

        # handle stopping the camera
        self._stopped = False

        self._cam.start(blocking=False)
        logging.info("Running the oak node")

    def _nn_callback(self, nndata: dai.NNData) -> None:
        logging.debug("New data packet in OakCam")
        raw = get_nn_data(nndata)
        val = raw[0]
        self._update_heading(val)

    def _left_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New left image in OakCam")
        img = frame.getCvFrame()
        self._leftpub.publish(self._bridge.cv2_to_imgmsg(img))

    def _right_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New right image in OakCam")
        img = frame.getCvFrame()
        self._rightpub.publish(self._bridge.cv2_to_imgmsg(img))

    def _depth_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New depth image in OakCam")
        img = frame.getCvFrame()
        self._depthpub.publish(self._bridge.cv2_to_imgmsg(img))

    def _disparity_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New disparity image in OakCam")
        img = frame.getCvFrame()
        self._disparitypub.publish(self._bridge.cv2_to_imgmsg(img))

    def _update_heading(self, heading: int) -> None:
        logging.debug(f"{time.perf_counter()} Updating heading with: {heading}")
        self._buffer.append(heading)
        # avg is between 0 and 4 floating point
        avg = float(np.mean(list(self._buffer)))

        # 90 is straight, 0 is hard left, 180 is hard right
        target_heading = Float32()
        target_heading.data = (avg / 2.0) - 1.0
        self._publisher.publish(target_heading)


def main(args=None):
    """
    Main function which exclusively launches the oak camera node
    """
    rclpy.init(args=args)
    cam = OakCam()
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()
