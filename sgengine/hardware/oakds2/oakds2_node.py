from typing import List, Tuple

import rclpy
import numpy as np

from ...abstract_node import AbstractNode
from .oakds2 import OakDS2


class OakDS2Node(AbstractNode, OakDS2):
    """Node for handling the OAKD-S2 camera"""

    def __init__(self) -> None:
        AbstractNode.__init__(self, "oakd_s2")
        OakDS2.__init__(self)

    def _handle_color_video_frame(self, frame: np.ndarray) -> None:
        """Handles the color video frame"""
        self.publish("color_camera", frame)

    def _handle_imu_data(self, imu_data: List[Tuple[np.ndarray, float]]) -> None:
        """Handles the IMU data"""
        self.publish("imu", imu_data)

    def _handle_depth_frame(self, frame: np.ndarray) -> None:
        """Handles the depth frame"""
        self.publish("depth_camera", frame)

    def _main(self):
        super().create_cam_rgb()
        super().create_imu()
        super().create_stereo()
        super().run()


def main(args=None):
    """
    Main function which exclusively launches the Oak node
    """
    rclpy.init(args=args)
    oak = OakDS2Node()
    rclpy.spin(oak)
    oak.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
