from threading import Thread

import cv2
import numpy as np

from oakds2 import OakD_S2


class OakDS2Test(OakD_S2):
    """Node for handling the OAKD-S2 camera"""

    def __init__(self) -> None:
        super().__init__()

    def _handle_color_video_frame(self, frame: np.ndarray) -> None:
        """Handles the color video frame"""
        cv2.imshow("color_camera", frame)
        if cv2.waitKey(1) == ord("q"):
            self.stop()

    def _handle_imu_data(self, rv_values: np.ndarray, rv_timestamp: float) -> None:
        """Handles the IMU data"""
        print(f"imu: rv_values:{rv_values}, rv_timestamp:{rv_timestamp}")

    def _handle_depth_frame(self, frame: np.ndarray) -> None:
        """Handles the depth frame"""
        cv2.imshow("depth_camera", frame)
        if cv2.waitKey(1) == ord("q"):
            self.stop()

    def main(self):
        super().create_cam_rgb()
        super().create_imu()
        super().create_stereo()

        async_thread = Thread(target=self._loop.run_forever())
        async_thread.start()

        super().run()
        self._loop.stop()
        async_thread.join()

if __name__ == "__main__":
    oakds2 = OakDS2Test()
    oakds2.main()
