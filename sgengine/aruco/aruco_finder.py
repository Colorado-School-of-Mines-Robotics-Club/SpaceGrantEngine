from typing import Tuple, Optional

import cv2
import numpy as np


class ArucoFinder:
    """
    Class for finding aruco markers in images and acquiring transformation matrices to them
    """

    def __init__(self, aruco_dict=cv2.aruco.DICT_4X4_100, marker_size=0.05, camera_matrix=None, dist_coeffs=None):
        """
        :param aruco_dict: The aruco dictionary to use for finding markers
        :param marker_size: The size of the markers in meters
        :param camera_matrix: The camera matrix to use for finding the transformation matrix
        :param dist_coeffs: The distortion coefficients to use for finding the transformation matrix
        """
        self._adict = cv2.arcuo.getPredefinedDictionary(aruco_dict)
        self._marker_size = marker_size
        self._camera_matrix = np.zeros((3, 3), dtype=np.float32) if camera_matrix is None else camera_matrix
        self._dist_coeffs = np.zeros((5, 1), dtype=np.float32) if dist_coeffs is None else dist_coeffs

    def get_transform(self, image: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """
        Gets the transformation matrix to the aruco marker (assumes only one is in view)
        :param image: The image to find the marker in
        :return: The transformation matrix to the marker and the rotation and translation vectors
        """
        corners, ids, _ = cv2.aruco.detectMarkers(image, self._adict)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self._marker_size, self._camera_matrix, self._dist_coeffs)

        try:
            rvec = rvecs[0]
            tvec = tvecs[0]
        except IndexError:
            return None

        R, _ = cv2.Rodrigues(rvec)  # Get equivalent 3x3 rotation matrix
        t = tvec.T  # Get translation as a 3x1 vector
        H = np.block([[R, t], [np.zeros((1, 3)), 1]])
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

    def get_transform_distance(self, image: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float]]:
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
    
    def get_transform_angle(self, image: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float]]:
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
    
    def get_transform_distance_angle(self, image: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float, float]]:
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
