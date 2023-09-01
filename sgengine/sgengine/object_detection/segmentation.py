from typing import Tuple

import numpy as np
import cv2
from sklearn.cluster import MiniBatchKMeans, KMeans, DBSCAN


def segment_image(
    image3d,
    method="minibatchkmeans",
    k=15,
    iterations=3,
    downscale=True,
    downscale_ratio=0.4,
    downscale_method="linear",
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """TODO"""
    cluster_method_dict = {
        "minibatchkmeans": MiniBatchKMeans,
        "kmeans": KMeans,
        "dbscan": DBSCAN,
    }
    assert method in cluster_method_dict
    cluster_method = cluster_method_dict[method]

    resize_method_dict = {
        "nearest": cv2.INTER_NEAREST,
        "linear": cv2.INTER_LINEAR,
        "area": cv2.INTER_AREA,
        "cubic": cv2.INTER_CUBIC,
    }
    assert downscale_method in resize_method_dict

    resized_image_3d = cv2.GaussianBlur(image3d, (3, 3), cv2.BORDER_DEFAULT)
    if downscale:
        width = int(image3d.shape[1] * downscale_ratio)
        height = int(image3d.shape[0] * downscale_ratio)
        dim = (width, height)
        resized_image_3d = cv2.resize(
            image3d, dim, resize_method_dict[downscale_method]
        )

    _, _, channels = resized_image_3d.shape
    vectorized = np.float32(resized_image_3d.reshape((-1, channels)))
    vectorized = vectorized[np.isfinite(vectorized)]
    # for i in range(len(vectorized)):
    #     for j in range(len(vectorized[i])):
    #         if not isinstance(vectorized[i][j], np.float32):
    #             vectorized[i][j] = 0
    #         if np.isnan(vectorized[i][j]):
    #             vectorized[i][j] = 0
    #         if np.isinf(vectorized[i][j]):
    #             vectorized[i][j] = 0

    cluster = cluster_method(n_clusters=k, n_init=iterations, random_state=0).fit(
        vectorized
    )
    centers, labels = cluster.cluster_centers_, cluster.labels_
    centers = np.uint8(centers)
    res = centers[labels.flatten()]
    res = res.reshape(resized_image_3d.shape)

    # resplit
    img = res

    result_image = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)

    return result_image, centers, labels
