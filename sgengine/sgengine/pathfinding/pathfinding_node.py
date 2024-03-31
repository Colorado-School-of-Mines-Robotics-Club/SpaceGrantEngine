import sys
import time

import depthai as dai
import open3d as o3d
import rclpy
import sensor_msgs.msg as sensor_msgs
from oakutils import ApiCamera
from oakutils.nodes import (
    create_color_camera,
    create_stereo_depth,
    create_xout,
    get_nn_point_cloud_buffer,
)
from oakutils.nodes.models import create_point_cloud
from oakutils.point_clouds import get_point_cloud_from_np_buffer
from oakutils.tools.parsing import (
    get_color_sensor_resolution_from_tuple,
    get_mono_sensor_resolution_from_tuple,
)
from rclpy.node import Node

from ..sg_logger import SG_Logger


class PathfindingNode(Node, SG_Logger):
    """Cam for handling depth-based obstancle avoidance"""

    def __init__(self) -> None:
        Node.__init__(self, "path_cam")
        SG_Logger.__init__(self)

        self.point_cloud_publisher = self.create_publisher(
            msg_type=sensor_msgs.PointCloud2,
            topic="pathfinding/point_cloud",
            qos_profile=10,
        )

        rgb_resolution = (1920, 1080)
        mono_resolution = (640, 400)
        use_left_mono = True
        print("Starting API Camera")
        self._oak = ApiCamera(
            primary_mono_left=use_left_mono,
            color_size=rgb_resolution,
            mono_size=mono_resolution,
        )
        print("Camera Initialized")

        self._cam = create_color_camera(
            self._oak.pipeline,
            fps=15,
            resolution=get_color_sensor_resolution_from_tuple(rgb_resolution),
        )
        self._xout_cam = create_xout(self._oak.pipeline, self._cam.video, "color")
        stereo, left, right = create_stereo_depth(
            self._oak.pipeline,
            resolution=get_mono_sensor_resolution_from_tuple(mono_resolution),
        )
        point_cloud, xin_xyz, start_pcl = create_point_cloud(
            self._oak.pipeline,
            stereo.depth,
            self._oak.calibration,
        )
        self._xout_point_cloud = create_xout(
            self._oak.pipeline, point_cloud.out, "point_cloud"
        )

        # add the basic display
        # oak.add_display("color")

        def pcl_callback(pcl: dai.NNData) -> None:
            """Use as callback for processing the pointcloud. Need a callback for api cam processing."""
            print("Point cloud!")
            pcl = get_nn_point_cloud_buffer(pcl)
            pcl: o3d.geometry.PointCloud = get_point_cloud_from_np_buffer(pcl)

            print(pcl)
            sys.exit(0)

            # ros_dtype = sensor_msgs.PointField.FLOAT32
            # dtype = np.float32
            # itemsize = np.dtype(dtype).itemsize

            # points = np.asarray(pcl.points)

            # data = points.astype(dtype).tobytes()

            # fields = [
            #     sensor_msgs.PointField(
            #         name=n, offset=i * itemsize, datatype=ros_dtype, count=1
            #     )
            #     for i, n in enumerate("xyz")
            # ]

            # header = std_msgs.Header(frame_id="map")

            # msg = sensor_msgs.PointCloud2(
            #     header=header,
            #     height=1,
            #     width=points.shape[0],
            #     is_dense=False,
            #     is_bigendian=False,
            #     fields=fields,
            #     point_step=(itemsize * 3),
            #     row_step=(itemsize * 3 * points.shape[0]),
            #     data=data,
            # )

            # self.point_cloud_publisher.publish(msg)

        self._oak.add_callback("point_cloud", pcl_callback)
        self._oak.add_device_call(
            start_pcl
        )  # start_pcl takes only an dai.Device, queue as such
        self._oak.start(blocking=False)

    def _run(self):
        while True:
            # logging.debug("PathCam new iteration")
            time.sleep(1)
        # pipeline = dai.Pipeline()
        # calibration = get_camera_calibration(
        #     rgb_size=(1920, 1080),
        #     mono_size=(640, 400),
        #     is_primary_mono_left=True,  # make sure to set primary to same as align_socket
        # )

        # # create the color camera node
        # cam = create_color_camera(pipeline, preview_size=(640, 480))
        # stereo, left, right = create_stereo_depth(pipeline)

        # _xout_rgb = create_xout(pipeline, cam.video, "rgb")
        # _xout_depth = create_xout(pipeline, stereo.depth, "depth")

        # with dai.Device(pipeline) as device:
        #     rgb_q: dai.DataOutputQueue = device.getOutputQueue("rgb")
        #     depth_q: dai.DataOutputQueue = device.getOutputQueue("depth")

        #     i = 0

        #     while True:
        #         i += 1
        #         in_rgb = rgb_q.get()
        #         in_depth = depth_q.get()
        #         rgb_frame = in_rgb.getCvFrame()
        #         depth_frame = in_depth.getFrame()

        #         logging.info(f"got frame {i}")

        #         point_cloud = get_point_cloud_from_rgb_depth_image(
        #             rgb_frame,
        #             depth_frame,
        #             calibration.primary.pinhole,
        #         )
        #         point_cloud = filter_point_cloud(
        #             point_cloud,
        #             voxel_size=0.01,
        #             nb_neighbors=60,
        #             std_ratio=0.1,
        #             downsample_first=False,
        #         )

        #         ros_dtype = sensor_msgs.PointField.FLOAT32
        #         dtype = np.float32
        #         itemsize = np.dtype(dtype).itemsize

        #         points = np.asarray(point_cloud.points)

        #         data = points.astype(dtype).tobytes()

        #         fields = [
        #             sensor_msgs.PointField(
        #                 name=n, offset=i * itemsize, datatype=ros_dtype, count=1
        #             )
        #             for i, n in enumerate("xyz")
        #         ]

        #         header = std_msgs.Header(frame_id="map")

        #         msg = sensor_msgs.PointCloud2(
        #             header=header,
        #             height=1,
        #             width=points.shape[0],
        #             is_dense=False,
        #             is_bigendian=False,
        #             fields=fields,
        #             point_step=(itemsize * 3),
        #             row_step=(itemsize * 3 * points.shape[0]),
        #             data=data,
        #         )

        #         self.point_cloud_publisher.publish(msg)


def main(args=None):
    """
    Main function which exclusively launches the pathfinding node
    """
    rclpy.init(args=args)
    pathfinding = PathfindingNode()
    rclpy.spin(pathfinding)
    pathfinding.destroy_node()
    rclpy.shutdown()
