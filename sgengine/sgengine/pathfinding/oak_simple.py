import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from oakutils import ApiCamera
from oakutils.nodes import create_stereo_depth, reate_neural_network, get_nn_data, create_xout

from sgengine_messages.msg import TwoFloat

from ..sg_logger import SG_Logger


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
        self._buffer: deque[int] = deque(maxlen=30)

        depth, left, right = create_stereo_depth(self._cam.pipeline)
        nn = create_neural_network(self._cam.pipeline, depth.depth, Path("data") / "simplePathfinding.blob")
        xout_nn = create_xout(self._cam.pipeline, nn.out, "nn")

        self._nodes = [
            depth,
            left,
            right,
            nn,
        ]
        self._publisher = self.create_publisher(TwoFloat, "pico/move_command", 10)
        self._cam.add_callback("nn", self._nn_callback)
        self._stopped = False

        self._run()

    def _nn_callback(self, nndata: dai.NNData) -> None:
        logging.debug("New data packet in PathCam")
        raw = get_nn_data(nndata, reshape_to=(1, 1))
        val = raw[0]
        self._update_heading(val)

    def _update_heading(self, heading: int) -> None:
        logging.debug(f"Updating heading with: {heading}")
        self._buffer.append(heading[0])
        # avg is between 0 and 4 floating point
        avg = float(np.mean(list(self._buffer)))

        move_cmd = TwoFloat()
        move_cmd.second = 0.5  # always use full speed
        move_cmd.first = (avg / 4.0) - 1.0  # between -1 and 1
        self._publisher.publish(move_cmd)

    def _run(self):
        while not self._stopped:
            logging.debug("PathCam new iteration")
            time.sleep(1)


def main(args=None):
    """
    Main function which exclusively launches the pathfinding node
    """
    rclpy.init(args=args)
    pathfinding = PathfindingNode()
    rclpy.spin(pathfinding)
    pathfinding.destroy_node()
    rclpy.shutdown()
