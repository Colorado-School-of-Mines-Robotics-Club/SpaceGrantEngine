from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rclpy
from sgengine_messages.msg import TwoFloat


def detect_heading(depth_map):
    """Detects the heading"""

    # fill out the edges so we don't use that data
    border_size = 50
    depth_map[:, 0:border_size] = 0
    depth_map[:, -border_size:-1] = 0
    depth_map[0:border_size, :] = 0
    depth_map[-border_size:-1, :] = 0

    min_depth = np.min(depth_map)
    max_depth = np.max(depth_map)

    # Blur source
    radius = 7
    depth_map = cv2.filter2D(depth_map, -1, np.ones((radius, radius), np.float32))

    laplacian = cv2.Laplacian(depth_map, cv2.CV_32F)
    n = np.linalg.norm(laplacian)
    laplacian /= n
    laplacian *= 255.0
    radius = 7
    # laplacian = cv2.filter2D(laplacian, -1, np.ones((radius,radius),np.float32))
    ma = np.max(laplacian)
    mi = np.min(laplacian)
    # print(f"max={ma}, min={mi}")

    # Bias the laplacian
    depth_scale = ((depth_map - min_depth) / (max_depth - min_depth)).astype(np.float32)
    # print(f"depthType={depthScale.dtype}, laplacianType={laplacian.dtype}")
    laplacian = laplacian + (1 - depth_scale)

    laplacian = np.where(laplacian < (mi + 0.3 * (ma - mi)), 0, laplacian)

    # Find best region
    binary = np.where(laplacian < 0.8, 0, 1)
    non_zeros = np.argwhere(binary > 0.5)
    distances = np.zeros((binary.shape[1]))
    c = int(binary.shape[0] / 2)

    if len(non_zeros) == 0:
        # just make points on left and right side to give us default heading
        non_zeros = np.zeros((2, 2), dtype=int)
        non_zeros[0] = (int(binary.shape[1]), c)
        non_zeros[1] = (0, c)

    for r in range(binary.shape[1]):
        parts = np.subtract(np.flip(non_zeros, axis=1), (r, c))
        parts = parts * parts
        euclid_dist = np.sqrt(np.sum(parts, axis=1))
        min_dist = np.min(euclid_dist)
        distances[r] = min_dist

    # Grayscale to bgr for debugging
    # laplacian = cv2.cvtColor(laplacian, cv2.COLOR_GRAY2BGR)

    # Debug non zero locations
    # for i,(x,y) in enumerate(nonZeros):
    #    laplacian = cv2.circle(laplacian, center=(y,x), radius=1, color=(0,1,0), thickness=1)

    # Debug distances vector
    # maxDist = np.max(distances)
    # minDist = np.min(distances)
    # print(f"MAXDIST={maxDist}, MINDIST={minDist}")
    # for i,d in enumerate(distances):
    #    scale = (d - minDist) / (maxDist - minDist)
    #    #print(f"i={i}, scale={scale}")
    #    laplacian = cv2.circle(laplacian, center=(int(i), int(c)), radius=1, color=(1.0-scale,scale,0), thickness=1)
    #    #laplacian = cv2.circle(laplacian, center=(int(i), int(c)), radius=1, color=(i / binary.shape[1],0,0), thickness=1)

    min_idx = np.argmax(distances)
    # print(f"minIdx={minIdx}")
    # laplacian = cv2.circle(laplacian, center=(int(minIdx), int(c)), radius=8, color=(0,0,0.5), thickness=3)

    # Rolling average of headings
    # headings = np.append(headings, minIdx)
    # historyLen = 8
    # print(f"h={headings}")
    # if len(headings) > historyLen:
    #    headings = headings[1 :]

    # avgHeading = np.average(headings)
    # laplacian = cv2.circle(laplacian, center=(int(avgHeading), int(c)), radius=14, color=(0,0,1), thickness=4)

    return min_idx


class PathfindingNode(Node):
    """Node for handling depth-based obstancle avoidance"""

    def __init__(self) -> None:
        Node.__init__(self, "pathfinding")

        self._history_len = 8
        self._headings = np.zeros(shape=(0, 1))
        self._bridge = CvBridge()

        def detect_callback(depth_map: Image) -> None:
            depth_map_cv = self._bridge.imgmsg_to_cv2(
                depth_map, desired_encoding="passthrough"
            )
            new_heading = detect_heading(depth_map_cv)
            self._headings = np.append(self._headings, new_heading)
            if len(self._headings) > self._history_len:
                self._headings = self._headings[1:]

            avg_heading = np.average(self._headings)
            norm_heading = float(avg_heading) / float(depth_map_cv.shape[1])
            norm_heading = norm_heading * 2.0 - 1.0  # [-1, 1]

            # Angular quantization
            thresh = 0.5
            if norm_heading < -thresh:
                norm_heading = -1
            elif norm_heading > thresh:
                norm_heading = 1
            else:
                norm_heading = 0

            move_cmd = TwoFloat()
            move_cmd.first = norm_heading  # angular
            move_cmd.second = 1 - abs(norm_heading)  # linear
            self._publisher.publish(move_cmd)

        self._subscription = self.create_subscription(
            Image, "/odom/depth", detect_callback, 10
        )
        self._publisher = self.create_publisher(TwoFloat, "pico/move_command", 10)

        print("Running Pathfinding")


def main(args=None):
    """
    Main function which exclusively launches the pathfinding node
    """
    rclpy.init(args=args)
    pathfinding = PathfindingNode()
    rclpy.spin(pathfinding)
    pathfinding.destroy_node()
    rclpy.shutdown()
