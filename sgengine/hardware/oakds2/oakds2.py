from abc import ABC, abstractmethod
from typing import Tuple, Dict, List, Optional, Union, Any
import asyncio
from collections import deque

try:
    import uvloop

    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())
except ImportError:
    pass

import depthai as dai
import numpy as np
import cv2


class OakD_S2(ABC):
    """Class for the DepthAI OAK-D Stereo Camera"""

    def __init__(self):
        # pipeline
        self._pipeline: dai.Pipeline = dai.Pipeline()

        # event loop
        self._loop = asyncio.new_event_loop()
        self._tasks: deque[asyncio.Task] = deque(maxlen=100)

        # storage for the nodes
        self._nodes: Dict[str, Tuple[dai.Node, dai.XLinkOut]] = {}

    @property
    def pipeline(self) -> dai.Pipeline:
        """Returns the pipeline"""
        return self._pipeline

    def create_cam_rgb(self) -> None:
        """Creates the RGB camera node"""

        cam_rgb = self._pipeline.create(dai.node.ColorCamera)
        xout_video = self._pipeline.create(dai.node.XLinkOut)
        xout_video.setStreamName("color_camera")
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setVideoSize(1920, 1080)
        xout_video.input.setBlocking(False)
        xout_video.input.setQueueSize(1)
        cam_rgb.video.link(xout_video.input)

        self._nodes["color_camera"] = (cam_rgb, xout_video)

    async def _async_handle_color_video_frame(self, frame: np.ndarray) -> None:
        """Handles the color video frame"""
        self._handle_color_video_frame(frame)

    @abstractmethod
    def _handle_color_video_frame(self, frame: np.ndarray) -> None:
        """Handles the color video frame"""
        pass

    def create_imu(self) -> None:
        """Creates the IMU node"""

        imu = self._pipeline.create(dai.node.IMU)
        imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
        imu.setBatchReportThreshold(1)
        imu.setMaxBatchReports(10)
        xout_imu = self._pipeline.create(dai.node.XLinkOut)
        xout_imu.setStreamName("imu")
        imu.out.link(xout_imu.input)

        self._nodes["imu"] = (imu, xout_imu)

    async def _async_handle_imu_data(
        self, rv_values: np.ndarray, rv_timestamp: float
    ) -> None:
        """Handles the IMU data"""
        self._handle_imu_data(rv_values, rv_timestamp)

    @abstractmethod
    def _handle_imu_data(self, rv_values: np.ndarray, rv_timestamp: float) -> None:
        """Handles the IMU data"""
        pass

    def _run(self) -> None:
        with dai.Device(self._pipeline) as device:

            video_queue = None
            if self._nodes["color_camera"] is not None:
                video_queue = device.getOutputQueue(
                    name="color_camera", maxSize=1, blocking=False
                )

            imu_queue = None
            base_transforms = None
            if self._nodes["imu"] is not None:
                imu_queue = device.getOutputQueue(
                    name="imu", maxSize=50, blocking=False
                )

            while True:
                if video_queue is not None:
                    video_frame = video_queue.get()
                    video_frame = video_frame.getCvFrame()

                    # do something with the video frame
                    self._tasks.append(
                        asyncio.ensure_future(
                            self._async_handle_color_video_frame(video_frame)
                        )
                    )

                if imu_queue is not None:
                    imu_data = imu_queue.get()
                    imu_packets = imu_data.packets
                    for packet in imu_packets:
                        rv_values = packet.rotationVector
                        rv_timestamp = rVvalues.getTimestampDevice()
                        if base_transforms is None:
                            base_transforms = rv_values
                        rv_values = rv_values - base_transforms

                        # do something with the imu data
                        self._tasks.append(
                            asyncio.ensure_future(
                                self._async_handle_imu_data(
                                    rv_values, rv_timestamp.total_seconds() * 1000.0
                                )
                            )
                        )
                        
    def run(self) -> None:
        """Runs the pipeline"""
        self._loop.run_until_complete(self._run())
