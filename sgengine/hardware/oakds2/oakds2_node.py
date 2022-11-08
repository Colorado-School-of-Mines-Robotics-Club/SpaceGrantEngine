from ....abstract_node import AbstractNode
from .oakds2 import OakD_S2


class OakDS2Node(AbstractNode, OakD_S2):
    """Node for handling the OAKD-S2 camera"""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)

    def _handle_color_video_frame(self, frame: np.ndarray) -> None:
        """Handles the color video frame"""
        self.publish("color_camera", frame)

    def _handle_imu_data(self, rv_values: np.ndarray, rv_timestamp: float) -> None:
        """Handles the IMU data"""
        self.publish("imu", (rv_values, rv_timestamp))

    def _main(self):
        super().create_cam_rgb()
        super().create_imu()
        super().run()
