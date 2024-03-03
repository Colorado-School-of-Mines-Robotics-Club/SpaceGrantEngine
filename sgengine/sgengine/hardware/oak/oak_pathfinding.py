import logging
import time
from collections import deque
from pathlib import Path

import depthai as dai
import numpy as np
import rclpy
from oakutils import ApiCamera
from oakutils.nodes import (create_neural_network, create_stereo_depth,
                            create_xout, get_nn_data)
from rclpy.node import Node
from std_msgs.msg import Float32

from ...sg_logger import SG_Logger


class PathCam(Node, SG_Logger):
    """Cam for handling depth-based obstancle avoidance"""

    def __init__(self) -> None:
        Node.__init__(self, "path_cam")
        SG_Logger.__init__(self)
        self._cam = ApiCamera(
            color_size=(1920, 1080),
            mono_size=(640, 400),
            primary_mono_left=True,
        )
        self._calibration = self._cam.calibration
        self._buffer: deque[int] = deque(maxlen=10)

        depth, left, right = create_stereo_depth(self._cam.pipeline)
        nn = create_neural_network(
            self._cam.pipeline, depth.depth, Path("data") / "simplePathfinding.blob"
        )
        xout_nn = create_xout(self._cam.pipeline, nn.out, "nn")

        self._nodes = [depth, left, right, nn, xout_nn]
        self._publisher = self.create_publisher(Float32, "oak/heading", 10)
        self._cam.add_callback("nn", self._nn_callback)
        self._stopped = False

        self._cam.start(blocking=False)
        self._run()

    def _nn_callback(self, nndata: dai.NNData) -> None:
        logging.debug("New data packet in PathCam")
        raw = get_nn_data(nndata)
        val = raw[0]
        self._update_heading(val)

    def _update_heading(self, heading: int) -> None:
        logging.debug(f"{time.perf_counter()} Updating heading with: {heading}")
        self._buffer.append(heading)
        # avg is between 0 and 4 floating point
        avg = float(np.mean(list(self._buffer)))

        # 90 is straight, 0 is hard left, 180 is hard right
        target_heading = Float32()
        target_heading.data = (avg / 2.0) - 1.0
        self._publisher.publish(target_heading)

    def _run(self):
        while not self._stopped:
            logging.debug("PathCam new iteration")
            time.sleep(1)


def main(args=None):
    """
    Main function which exclusively launches the pathfinding node
    """
    rclpy.init(args=args)
    pathfinding = PathCam()
    rclpy.spin(pathfinding)
    pathfinding.destroy_node()
    rclpy.shutdown()
