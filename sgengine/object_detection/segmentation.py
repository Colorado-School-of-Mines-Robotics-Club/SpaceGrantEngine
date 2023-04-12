# Libraries
import numpy as np
import cv2
from openVO import StereoCamera, StereoOdometer, drawPoseOnImage
from typing import Tuple, Dict
import pickle
from sklearn.cluster import MiniBatchKMeans, KMeans, DBSCAN


# loads all files from data that the robot needs
def loadCalibrationFiles(calibrationPath) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[str, np.ndarray]]:
    # one time file loading for the camera intrinsic matrices and undistortion coeff
    leftK = np.load(calibrationPath + "leftK.npy")
    rightK = np.load(calibrationPath + "rightK.npy")
    leftDistC = np.load(calibrationPath + "leftDistC.npy")
    rightDistC = np.load(calibrationPath + "rightDistC.npy")
    rectParams = pickle.load(open(calibrationPath + "rectParams.p", "rb"))

    return leftK, rightK, leftDistC, rightDistC, rectParams


def makeOdometer(args: Tuple) -> StereoOdometer:
    cameraArgs, odometerParams = args
    return StereoOdometer(StereoCamera(*cameraArgs), **odometerParams)


def segmentImage(image3d=None, method='minibatchkmeans', K=15, iterations=3, downscale=True,
                 downscaleRatio=0.4, downscaleMethod='linear')\
        -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    cluster_method_dict = {
        'minibatchkmeans': MiniBatchKMeans,
        'kmeans': KMeans,
        'dbscan': DBSCAN
    }
    assert method in cluster_method_dict
    cluster_method = cluster_method_dict[method]

    resize_method_dict = {
        'nearest': cv2.INTER_NEAREST,
        'linear': cv2.INTER_LINEAR,
        'area': cv2.INTER_AREA,
        'cubic': cv2.INTER_CUBIC
    }
    assert downscaleMethod in resize_method_dict

    if image3d is not None:
        resized_image_3d = cv2.GaussianBlur(image3d, (3, 3), cv2.BORDER_DEFAULT)
        if downscale:
            width = int(image3d.shape[1] * downscaleRatio)
            height = int(image3d.shape[0] * downscaleRatio)
            dim = (width, height)
            resized_image_3d = cv2.resize(image3d, dim, resize_method_dict[downscaleMethod])

    _, _, channels = resized_image_3d.shape
    vectorized = np.float32(resized_image_3d.reshape((-1, channels)))
    for i in range(len(vectorized)):
        for j in range(len(vectorized[i])):
            if not isinstance(vectorized[i][j], np.float32):
                vectorized[i][j] = 0
            if np.isnan(vectorized[i][j]):
                vectorized[i][j] = 0
            if np.isinf(vectorized[i][j]):
                vectorized[i][j] = 0

    cluster = cluster_method(n_clusters=K, n_init=iterations, random_state=0).fit(vectorized)
    centers, labels = cluster.cluster_centers_, cluster.labels_
    centers = np.uint8(centers)
    res = centers[labels.flatten()]
    res = res.reshape(resized_image_3d.shape)

    # resplit
    img = res

    result_image = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)

    return result_image, centers, labels


if __name__ == "__main__":
    odometer_params = {
      "nfeatures": 500,
      "match_threshold": 0.9,
      "rigidity_threshold": 0.06,
      "outlier_threshold": 0.02,
      "preprocessed_frames": True
    }
    sgbm_params = {
        "minDisparity": 0,
        "numDisparities": 80,
        "blockSize": 1,
        "P1": 0,
        "P2": 135,
        "disp12MaxDiff": 2,
        "preFilterCap": 0,
        "uniquenessRatio": 0,
        "speckleWindowSize": 50,
        "speckleRange": 2
    }
    frameSize = [640, 480]
    leftK, rightK, leftDistC, rightDistC, rectParams = loadCalibrationFiles("sgengine/object_detection/competition1/")
    cameraArgs = (leftK, leftDistC, rightK, rightDistC, rectParams, sgbm_params, frameSize)
    odometer = makeOdometer((cameraArgs, odometer_params))

    video_left = cv2.VideoCapture("sgengine/object_detection/RocksTest/stereo_left.avi")
    video_right = cv2.VideoCapture("sgengine/object_detection/RocksTest/stereo_right.avi")

    if video_left.isOpened() is False:
        print("Error opening video stream or file")

    # Read until video is completed
    while video_left.isOpened():
        # Capture frame-by-frame
        ret1, frame_left = video_left.read()
        ret2, frame_right = video_right.read()
        # Check if both frames are valid
        if ret1 is True and ret2 is True:
            # Update odometer
            odometer.update(frame_left, frame_right)
            im3d = odometer.current_3d
            print(im3d)

            # Run segmentation
            result_image, centers, labels = segmentImage(im3d)

            # Show the resulting image
            cv2.imshow('image', result_image)
            cv2.waitKey(0)
        # Break the loop
        else:
            break

    # When everything done, release the video capture object
    video_left.release()
    video_right.release()
