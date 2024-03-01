import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from oakutils import ApiCamera
from oakutils.nodes import create_stereo_depth, reate_neural_network, get_nn_data, create_xout

from sgengine_messages.msg import TwoFloat


class PathCam(Node):
    """Node for handling depth-based obstancle avoidance"""

    def __init__(self) -> None:
        Node.__init__(self, "pathfinding")
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
        self._cam.add_callback("nn", detect_callback)
    
    def _update_heading(self, heading: np.ndarray) -> None:
        self._buffer.append(heading[0])
        # avg is between 0 and 4 floating point
        avg = float(np.mean(list(self._buffer)))

        move_cmd = TwoFloat()
        move_cmd.second = 0.5  # always use full speed
        move_cmd.first = (avg / 4.0) - 1.0  # between -1 and 1
        self._publisher.publish(move_cmd)


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

        self._publisher = self.create_publisher(TwoFloat, "pico/move_command", 10)

        self._pose = None

        self._stopped = False

        # atexit.register(self._stop)

        self._run()

    def _stop(self) -> None:
        self._stopped = True
        self._cam.stop()

    def _run(self) -> None:
        print("Running the pathfinding node")
        while not self._stopped:
            

        self._cam.stop()


def main(args=None):
    """
    Main function which exclusively launches the pathfinding node
    """
    rclpy.init(args=args)
    pathfinding = PathfindingNode()
    rclpy.spin(pathfinding)
    pathfinding.destroy_node()
    rclpy.shutdown()
