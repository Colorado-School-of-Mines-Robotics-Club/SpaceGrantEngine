import rosbag
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from odometry.odometry_node import OdometryNode
from sgengine_messages.msg import RPYXYZ

class RosbagUtils(Node):
    """Node for saving camera images to a rosbag in order to be reused for 
    testing without needing to connect the camera."""

    def __init__(self, filename) -> None:
        Node.__init__(self, "rosbag_utils")
        self._filename = filename
        self._bag = rosbag.Bag(self._filename, 'w')
        self._pose_subscriber = self.create_subscription(RPYXYZ, "/odom/rpy_xyz", self.save_pose, 10)
        self._depth_subscriber = self.create_publisher(Image, "/odom/depth", self.save_depth, 10)
        self._rgb_subscriber = self.create_publisher(Image, "/odom/rgb", self.save_rgb, 10)
    
    def save_pose(self, pose: RPYXYZ):
        try:
            self._bag.write('pose', pose)
        except Exception as e:
            print("Something went wrong when saving to a rosbag: ", e)

    def save_depth(self, depth_img: Image):
        try:
            self._bag.write('depth', depth_img)
        except Exception as e:
            print("Something went wrong when saving to a rosbag: ", e)
    
    def save_rgb(self, rgb_img: Image):
        try:
            self._bag.write('rgb', rgb_img)
        except Exception as e:
            print("Something went wrong when saving to a rosbag: ", e)

    def close_bag(self):
        self._bag.close()

def main(args=None):
    """Main function which launches the RosbagUtils node and Odometry node."""
    rclpy.init(args=args)
    odometer = OdometryNode()
    rosbag_utils = RosbagUtils()
    rclpy.spin(odometer)
    rclpy.spin(rosbag_utils)
    rosbag_utils.close_bag()
    odometer.destroy_node()
    rosbag_utils.destroy_node()
    rclpy.shutdown()

