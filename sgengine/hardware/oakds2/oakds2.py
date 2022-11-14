from abc import ABC, abstractmethod
from typing import Tuple, Dict, List

import depthai as dai
import numpy as np


class OakDS2(ABC):
    """Class for the DepthAI OAK-D Stereo Camera"""

    def __init__(self):
        # pipeline
        self._pipeline: dai.Pipeline = dai.Pipeline()

        # storage for the nodes
        self._nodes: Dict[str, Tuple[dai.Node, dai.XLinkOut]] = {}

        # stop condition
        self._stopped: bool = False

    @property
    def pipeline(self) -> dai.Pipeline:
        """Returns the pipeline"""
        return self._pipeline

    def stop(self) -> None:
        """Stops the run loop and event loop"""
        self._stopped = True

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

    @abstractmethod
    def _handle_imu_data(self, imu_data: List[Tuple[np.ndarray, float]]) -> None:
        """Handles the IMU data"""
        pass

    def create_stereo(
        self, extended_disparity=False, subpixel=False, lr_check=True
    ) -> None:
        """Creates the stereo node"""

        # Define sources and outputs
        mono_left = self._pipeline.create(dai.node.MonoCamera)
        mono_right = self._pipeline.create(dai.node.MonoCamera)
        depth = self._pipeline.create(dai.node.StereoDepth)
        xout_depth = self._pipeline.create(dai.node.XLinkOut)

        xout_depth.setStreamName("disparity")

        # Properties
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
        depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
        depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        depth.setLeftRightCheck(lr_check)
        depth.setExtendedDisparity(extended_disparity)
        depth.setSubpixel(subpixel)

        config = depth.initialConfig.get()
        config.postProcessing.speckleFilter.enable = False
        config.postProcessing.speckleFilter.speckleRange = 50
        config.postProcessing.temporalFilter.enable = True
        config.postProcessing.spatialFilter.enable = True
        config.postProcessing.spatialFilter.holeFillingRadius = 2
        config.postProcessing.spatialFilter.numIterations = 1
        config.postProcessing.thresholdFilter.minRange = 400
        config.postProcessing.thresholdFilter.maxRange = 15000
        config.postProcessing.decimationFilter.decimationFactor = 1
        depth.initialConfig.set(config)

        # Linking
        mono_left.out.link(depth.left)
        mono_right.out.link(depth.right)
        depth.disparity.link(xout_depth.input)

        self._nodes["stereo"] = (depth, xout_depth)
        self._nodes["mono_left"] = (mono_left, None)
        self._nodes["mono_right"] = (mono_right, None)

    @abstractmethod
    def _handle_depth_frame(self, frame: np.ndarray) -> None:
        """Handles the depth frame"""
        pass

    def _run(self) -> None:
        with dai.Device(self._pipeline) as device:

            video_queue = None
            if self._nodes["color_camera"] is not None:
                video_queue = device.getOutputQueue(
                    name="color_camera", maxSize=1, blocking=False
                )

            imu_queue = None
            base_timestamp = None
            if self._nodes["imu"] is not None:
                imu_queue = device.getOutputQueue(
                    name="imu", maxSize=50, blocking=False
                )

            depth_queue = None
            if self._nodes["stereo"] is not None:
                depth_queue = device.getOutputQueue(
                    name="disparity", maxSize=1, blocking=False
                )

            while not self._stopped:
                if video_queue is not None:
                    video_frame = video_queue.get()
                    video_frame = video_frame.getCvFrame()

                    # do something with the video frame
                    self._handle_color_video_frame(video_frame)

                if imu_queue is not None:
                    imu_data = imu_queue.get()
                    imu_packets = imu_data.packets
                    imu_list_data: List[Tuple] = []
                    for packet in imu_packets:
                        rv_values = packet.rotationVector
                        rv_timestamp = rv_values.getTimestampDevice()
                        if base_timestamp is None:
                            base_timestamp = rv_timestamp
                        rv_timestamp = rv_timestamp - base_timestamp
                        imu_list_data.append(
                            (rv_values, rv_timestamp.total_seconds() * 1000.0)
                        )

                    # do something with the imu data
                    self._handle_imu_data(imu_list_data)

                if depth_queue is not None:
                    depth_frame = depth_queue.get()
                    depth_frame = depth_frame.getFrame()

                    # do something with the depth frame
                    self._handle_depth_frame(depth_frame)

    def run(self) -> None:
        """Runs the pipeline"""
        self._run()
