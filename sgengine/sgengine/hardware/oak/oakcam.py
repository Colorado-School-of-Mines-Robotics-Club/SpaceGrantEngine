import logging
import pickle
import time
from collections import deque
from pathlib import Path

import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from oakutils import ApiCamera
from oakutils.aruco import ArucoStream
from oakutils.nodes import (
    create_color_camera,
    create_neural_network,
    create_stereo_depth,
    create_xout,
    get_nn_data,
)
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

from sgengine_messages.msg import Aruco, ArucoArray

from ...sg_logger import SG_Logger


class OakCam(Node, SG_Logger):
    """Cam for handling depth-based obstancle avoidance"""

    def __init__(self) -> None:
        Node.__init__(self, "oak")
        SG_Logger.__init__(self)
        self._cam = ApiCamera(
            color_size=(1920, 1080),
            mono_size=(640, 400),
            primary_mono_left=True,
        )
        self._calibration = self._cam.calibration
        self._arucostream = ArucoStream(calibration=self._calibration.rgb)

        # write the calibration data to a file (MUST NOT CONTAIN OPEN3D OBJECTS)
        calibration_file = Path("tmp") / "calibration.pkl"
        Path("tmp").mkdir(parents=True, exist_ok=True)
        with calibration_file.open("wb") as f:
            pickle.dump(self._calibration, f, pickle.HIGHEST_PROTOCOL)

        self._buffer: deque[int] = deque(maxlen=10)

        self._bridge = CvBridge()

        color = create_color_camera(self._cam.pipeline, fps=5, preview_size=(640, 480))
        depth, left, right = create_stereo_depth(self._cam.pipeline, fps=5)
        nn = create_neural_network(
            self._cam.pipeline, depth.depth, Path("data") / "simplePathfinding.blob"
        )

        # create xout links for the data streams we want to publish
        xout_nn = create_xout(self._cam.pipeline, nn.out, "nn")
        xout_color = create_xout(self._cam.pipeline, color.preview, "color")
        xout_rectleft = create_xout(self._cam.pipeline, depth.rectifiedLeft, "rectleft")
        xout_rectright = create_xout(
            self._cam.pipeline, depth.rectifiedRight, "rectright"
        )
        xout_depth = create_xout(self._cam.pipeline, depth.depth, "depth")
        xout_disparity = create_xout(self._cam.pipeline, depth.disparity, "disparity")

        # save the nodes
        self._nodes = [
            color,
            depth,
            left,
            right,
            nn,
            xout_nn,
            xout_color,
            xout_rectleft,
            xout_rectright,
            xout_depth,
            xout_disparity,
        ]

        # create any output publishers
        self._publisher = self.create_publisher(Float32, "oak/simple_heading", 10)
        self._colorpub = self.create_publisher(Image, "oak/color_image", 10)
        self._leftpub = self.create_publisher(Image, "oak/left_image", 10)
        self._rightpub = self.create_publisher(Image, "oak/right_image", 10)
        self._depthpub = self.create_publisher(Image, "oak/depth_image", 10)
        self._depth_info_pub = self.create_publisher(
            CameraInfo, "oak/depth_camera_info", 10
        )
        self._disparitypub = self.create_publisher(Image, "oak/disparity_image", 10)

        # aruco publishers
        self._aruco_posepub = self.create_publisher(ArucoArray, "oak/aruco", 10)

        # add the callbacks
        self._cam.add_callback("nn", self._nn_callback)
        self._cam.add_callback("color", self._color_callback)
        self._cam.add_callback("rectleft", self._left_callback)
        self._cam.add_callback("rectright", self._right_callback)
        self._cam.add_callback("depth", self._depth_callback)
        self._cam.add_callback("disparity", self._disparity_callback)

        # handle stopping the camera
        self._stopped = False

        self._cam.start(blocking=False)
        logging.info("Running the oak node")

    def _nn_callback(self, nndata: dai.NNData) -> None:
        logging.debug("New data packet in OakCam")
        raw = get_nn_data(nndata)
        val = raw[0]
        self._update_heading(val)

    def _aruco_callback(self, img: np.ndarray) -> None:
        logging.debug("New aruco data in OakCam")
        markers = self._arucostream.find(img, rectified=False)
        if len(markers) == 0:
            return

        ros_markers = ArucoArray()

        for marker in markers:
            m_id, transform, rvec, tvec, corners = marker

            distance = np.linalg.norm(tvec)
            rotation_matrix = transform[:3, :3]
            r11, r12, r13 = rotation_matrix[0]
            r21, r22, r23 = rotation_matrix[1]
            r31, r32, r33 = rotation_matrix[2]

            # Calculate the scalar component
            qw = np.sqrt(1 + r11 + r22 + r33) / 2
            qx = (r32 - r23) / (4 * qw)
            qy = (r13 - r31) / (4 * qw)
            qz = (r21 - r12) / (4 * qw)

            aruco_marker = Aruco()
            aruco_marker.marker.translation.x = tvec[0]
            aruco_marker.marker.translation.y = tvec[1]
            aruco_marker.marker.translation.z = tvec[2]
            aruco_marker.marker.rotation.w = qw
            aruco_marker.marker.rotation.x = qx
            aruco_marker.marker.rotation.y = qy
            aruco_marker.marker.rotation.z = qz
            aruco_marker.distance = distance
            aruco_marker.id = int(m_id)

            ros_markers.markers.append(aruco_marker)

        self._aruco_posepub.publish(ros_markers)

    def _color_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New color image in OakCam")
        img = frame.getCvFrame()
        self._colorpub.publish(self._bridge.cv2_to_imgmsg(img))
        self._aruco_callback(img)

    def _left_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New left image in OakCam")
        img = frame.getCvFrame()
        self._leftpub.publish(self._bridge.cv2_to_imgmsg(img))

    def _right_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New right image in OakCam")
        img = frame.getCvFrame()
        self._rightpub.publish(self._bridge.cv2_to_imgmsg(img))

    def _depth_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New depth image in OakCam")
        img = frame.getCvFrame()
        depth_image_msg = self._bridge.cv2_to_imgmsg(img)
        self._depthpub.publish(depth_image_msg)

        camera_info_msg = CameraInfo()
        camera_info_msg.header = depth_image_msg.header
        camera_info_msg.header.stamp = depth_image_msg.header.stamp

        camera_info_msg.height = self._calibration.stereo.left.size[1]
        camera_info_msg.width = self._calibration.stereo.left.size[0]

        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.d = self._calibration.stereo.left.D.flatten().tolist()

        camera_info_msg.k = [0.0] * 9
        camera_info_msg.k[0] = self._calibration.stereo.left.fx
        camera_info_msg.k[2] = self._calibration.stereo.left.cx
        camera_info_msg.k[4] = self._calibration.stereo.left.fy
        camera_info_msg.k[5] = self._calibration.stereo.left.cy
        camera_info_msg.k[8] = 1.0

        camera_info_msg.r = self._calibration.stereo.left.R.flatten().tolist()

        camera_info_msg.p = [0.0] * 12
        camera_info_msg.p[0] = self._calibration.stereo.left.fx
        camera_info_msg.p[2] = self._calibration.stereo.left.cx
        camera_info_msg.p[5] = self._calibration.stereo.left.fy
        camera_info_msg.p[6] = self._calibration.stereo.left.cy
        camera_info_msg.p[10] = 1.0

        # camera_info_msg.binning_x = 0
        # camera_info_msg.binning_y = 0

        # camera_info_msg.roi.x_offset = 0
        # camera_info_msg.roi.y_offset = 0
        # camera_info_msg.roi.height = 0
        # camera_info_msg.roi.width = 0
        # camera_info_msg.roi.do_rectify = False

        self._depth_info_pub.publish(camera_info_msg)

    def _disparity_callback(self, frame: dai.ImgFrame) -> None:
        logging.debug("New disparity image in OakCam")
        img = frame.getCvFrame()
        self._disparitypub.publish(self._bridge.cv2_to_imgmsg(img))

    def _update_heading(self, heading: int) -> None:
        logging.debug(f"{time.perf_counter()} Updating heading with: {heading}")
        self._buffer.append(heading)
        # avg is between 0 and 4 floating point
        avg = float(np.mean(list(self._buffer)))

        # 90 is straight, 0 is hard left, 180 is hard right
        target_heading = Float32()
        target_heading.data = (avg / 2.0) - 1.0
        self._publisher.publish(target_heading)


def main(args=None):
    """
    Main function which exclusively launches the oak camera node
    """
    rclpy.init(args=args)
    cam = OakCam()
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()
