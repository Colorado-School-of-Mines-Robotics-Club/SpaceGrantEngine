from aruco_finder import ArucoFinder

import numpy as np


K = np.array(
    [
        [3060.68701171875, 0.0, 1997.737548828125],
        [0.0, 3060.68701171875, 860.2412109375],
        [0.0, 0.0, 1.0],
    ]
)

af = ArucoFinder(marker_size=0.2, camera_matrix=K)

import os
import time
from threading import Thread

import cv2
import depthai as dai


class Camera:
    def __init__(self):
        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define source and output
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.xoutVideo = self.pipeline.create(dai.node.XLinkOut)

        self.xoutVideo.setStreamName("video")

        # Properties
        self.camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setVideoSize(1920, 1080)

        self.xoutVideo.input.setBlocking(False)
        self.xoutVideo.input.setQueueSize(1)

        # Linking
        self.camRgb.video.link(self.xoutVideo.input)

        # OpenCV VideoWriter
        # create the directory for this data
        self.data_dir = os.path.join(os.getcwd(), f"data/{int(time.time())}")
        # os.mkdir(self.data_dir)
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        self.writer = cv2.VideoWriter(
            os.path.join(self.data_dir, "output.avi"), fourcc, 30.0, (1920, 1080)
        )

        # Define sources and outputs
        self.imu = self.pipeline.create(dai.node.IMU)
        self.xlinkOut = self.pipeline.create(dai.node.XLinkOut)

        self.xlinkOut.setStreamName("imu")

        # enable ROTATION_VECTOR at 400 hz rate
        self.imu.enableIMUSensor(dai.IMUSensor.ROTATION_VECTOR, 400)
        # it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
        # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
        self.imu.setBatchReportThreshold(1)
        # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
        # if lower or equal to batchReportThreshold then the sending is always blocking on device
        # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
        self.imu.setMaxBatchReports(10)

        # Link plugins IMU -> XLINK
        self.imu.out.link(self.xlinkOut.input)

    def record(self):
        # Connect to device and start pipeline
        with dai.Device(self.pipeline) as device:
            # with open(os.path.join(self.data_dir, "imu.txt"), "x") as f:

            video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

            def timeDeltaToMilliS(delta) -> float:
                return delta.total_seconds() * 1000

            # Output queue for imu bulk packets
            imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
            baseTs = None

            while True:
                videoIn = video.get()
                frame = videoIn.getCvFrame()

                # write frame
                # self.writer.write(frame)

                imuData = (
                    imuQueue.get()
                )  # blocking call, will wait until a new data has arrived

                imuPackets = imuData.packets
                for imuPacket in imuPackets:
                    rVvalues = imuPacket.rotationVector

                    rvTs = rVvalues.getTimestampDevice()
                    if baseTs is None:
                        baseTs = rvTs
                    rvTs = rvTs - baseTs

                    imuF = "{:.06f}"
                    tsF = "{:.03f}"

                    data_line = f"ts: {tsF.format(timeDeltaToMilliS(rvTs))}, "
                    data_line += f"i: {imuF.format(rVvalues.i)}, "
                    data_line += f"j: {imuF.format(rVvalues.j)}, "
                    data_line += f"k: {imuF.format(rVvalues.k)}, "
                    data_line += f"real: {imuF.format(rVvalues.real)}, "
                    data_line += (
                        f"acc: {imuF.format(rVvalues.rotationVectorAccuracy)}\n"
                    )

                    # f.write(data_line)

                # Get BGR frame from NV12 encoded video frame to show with opencv
                # Visualizing the frame on slower hosts might have overhead
                # cv2.imshow("video", frame)
                _ = af.get_transform(frame, draw=True)

        # self.writer.release()


if __name__ == "__main__":
    cam = Camera()
    cam.record()
