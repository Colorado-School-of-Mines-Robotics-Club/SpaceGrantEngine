from typing import Tuple, Optional

import cv2
import numpy as np
import time
from pathlib import Path
import depthai as dai
import pyvirtualcam
import math


class ArucoFinder:
    """
    Class for finding aruco markers in images and acquiring transformation matrices to them
    """

    def __init__(
        self,
        aruco_dict=cv2.aruco.DICT_4X4_100,
        marker_size=0.05,
        camera_matrix=None,
        dist_coeffs=None,
    ):
        """
        :param aruco_dict: The aruco dictionary to use for finding markers
        :param marker_size: The size of the markers in meters
        :param camera_matrix: The camera matrix to use for finding the transformation matrix
        :param dist_coeffs: The distortion coefficients to use for finding the transformation matrix
        """
        self._adict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self._marker_size = marker_size
        self._camera_matrix = (
            np.zeros((3, 3), dtype=np.float32)
            if camera_matrix is None
            else camera_matrix
        )
        self._dist_coeffs = (
            np.zeros((5, 1), dtype=np.float32) if dist_coeffs is None else dist_coeffs
        )

    def get_transform(
        self, image: np.ndarray, draw=False
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """
        Gets the transformation matrix to the aruco marker (assumes only one is in view)
        :param image: The image to find the marker in
        :return: The transformation matrix to the marker and the rotation and translation vectors
        """
        corners, ids, _ = cv2.aruco.detectMarkers(image, self._adict)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self._marker_size, self._camera_matrix, self._dist_coeffs
        )

        try:
            rvec = rvecs[0]
            tvec = tvecs[0]
        except IndexError:
            return None

        R, _ = cv2.Rodrigues(rvec)  # Get equivalent 3x3 rotation matrix
        t = tvec.T  # Get translation as a 3x1 vector
        H = np.block([[R, t], [np.zeros((1, 3)), 1]])

        if draw:
            cv2.aruco.drawDetectedMarkers(image, corners)
            cv2.aruco.drawAxes(
                image, self._camera_matrix, self._dist_coeffs, rvec, tvec, 0.01
            )

            cv2.imshow("ArUco Markers")
            cv2.waitKey(1)

        return H, rvec, tvec

    @staticmethod
    def _get_distance(tvec: np.ndarray) -> float:
        """
        Gets the distance to the marker from the translation vector
        """
        return np.linalg.norm(tvec)

    @staticmethod
    def _get_angle(tvec: np.ndarray) -> float:
        """
        Gets the angle to the marker from the translation vector
        """
        return np.arctan2(tvec[0], tvec[2])

    def get_transform_distance(
        self, image: np.ndarray
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float]]:
        """
        Gets the distance to the marker with the transformation matrix
        :param image: The image to find the marker in
        :return: The transformation matrix to the marker, the rotation and translation vectors, and the distance to the marker
        """
        try:
            H, rvec, tvec = self.get_transform(image)

            return H, rvec, tvec, self._get_distance(tvec)
        except TypeError:
            return None

    def get_transform_angle(
        self, image: np.ndarray
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float]]:
        """
        Gets the angle to the marker with the transformation matrix
        :param image: The image to find the marker in
        :return: The transformation matrix to the marker, the rotation and translation vectors, and the angle to the marker
        """
        try:
            H, rvec, tvec = self.get_transform(image)

            return H, rvec, tvec, self._get_angle(tvec)
        except TypeError:
            return None

    def get_transform_distance_angle(
        self, image: np.ndarray
    ) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float, float]]:
        """
        Gets the distance and angle to the marker with the transformation matrix
        :param image: The image to find the marker in
        :return: The transformation matrix to the marker, the rotation and translation vectors, the distance to the marker, and the angle to the marker
        """
        try:
            H, rvec, tvec = self.get_transform(image)

            return H, rvec, tvec, self._get_distance(tvec), self._get_angle(tvec)
        except TypeError:
            return None

    def get_distance_vector(self, image: np.ndarray):
        """
        Returns the x,y,z away from the marker
        """
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            image, arucoDict, parameters=arucoParams
        )
        color = (255, 255, 255)

        K = np.array(
            [
                [3060.68701171875, 0.0, 1997.737548828125],
                [0.0, 3060.68701171875, 860.2412109375],
                [0.0, 0.0, 1.0],
            ]
        )

        markerSizeInCM = 0.05

        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, markerSizeInCM, K, None
        )

        return tvec


def getMonoCamera(pipeline, isLeft):
    # Configure mono camera
    mono = pipeline.createMonoCamera()

    # Set Camera Resolution
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    if isLeft:
        # Get left camera
        mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
    else:
        # Get right camera
        mono.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    return mono


def getFrame(queue):
    # Get frame from queue
    frame = queue.get()
    # Convert frame to OpenCV format and return
    return frame.getCvFrame()


def main():
    # Code to test the aruco detection class
    markerSizeInCM = 0.05

    arucoFinder = ArucoFinder()

    pipeline = dai.Pipeline()

    mono = pipeline.createMonoCamera()

    xout = pipeline.createXLinkOut()
    xout.setStreamName("left")
    mono.out.link(xout.input)

    with dai.Device(pipeline) as device:
        queue = device.getOutputQueue(name="left")
        frame = queue.get()
        imOut = frame.getCvFrame()

        while True:
            frame = queue.get()
            imOut = frame.getCvFrame()

            print(arucoFinder.get_distance_vector(imOut))

            # Check for keyboard input
            key = cv2.waitKey(1)
            if key == ord("q"):
                # Quit when q is pressed
                break
            elif key == ord("t"):
                # Toggle display when t is pressed
                sideBySide = not sideBySide


if __name__ == "__main__":
    main()
