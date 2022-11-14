from typing import List, Tuple

import cv2
import numpy as np

try:
    from oakds2 import OakDS2  # type: ignore
except ImportError:
    from sgengine.hardware.oakds2 import OakDS2


class OakDS2Test(OakDS2):
    """Node for handling the OAKD-S2 camera"""

    def __init__(self) -> None:
        super().__init__()

    def _handle_color_video_frame(self, frame: np.ndarray) -> None:
        """Handles the color video frame"""
        cv2.imshow("color_camera", frame)
        if cv2.waitKey(1) == ord("q"):
            self.stop()

    def _handle_imu_data(self, imu_data: List[Tuple[np.ndarray, float]]) -> None:
        """Handles the IMU data"""
        print(f"imu: {imu_data}")

    def _handle_depth_frame(self, frame: np.ndarray) -> None:
        """Handles the depth frame"""
        cv2.imshow("depth_camera", frame)
        if cv2.waitKey(1) == ord("q"):
            self.stop()

    def main(self):
        """Main method"""
        self.create_cam_rgb()
        self.create_imu()
        self.create_stereo()
        self.run()

    def stop(self):
        """Stops the run loop"""
        super().stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    oakds2 = OakDS2Test()
    oakds2.main()
