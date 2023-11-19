import rosbag
import rclpy
import sys
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
    
    def write_bag(self):
        self._pose_subscriber = self.create_subscription(RPYXYZ, "/odom/rpy_xyz", self.save_pose, 10)
        self._depth_subscriber = self.create_subscription(Image, "/odom/depth", self.save_depth, 10)
        self._rgb_subscriber = self.create_subscription(Image, "/odom/rgb", self.save_rgb, 10)
    
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

    def read_bag(self):
        self._pose_publisher = self.create_publisher(RPYXYZ, "/odom/rpy_xyz", 10)
        self._depth_publisher = self.create_subscription(Image, "/odom/depth", self.save_depth, 10)
        self._rgb_publisher = self.create_subscription(Image, "/odom/rgb", self.save_rgb, 10)

        for topic, msg in self._bag.read_messages(topics=['pose', 'depth', 'rgb']):
            if topic == '/odom/rpy_xyz':
                self._pose_publisher.publish(msg)  # Publish to pose topic
            elif topic == '/odom/depth':
                self._depth_publisher.publish(msg)  # Publish to depth image topic
            else:
                self._rgb_publisher.publish(msg)  # Publish to rgb image topic

def main(args=None, read=True):
    """Main function which launches the RosbagUtils node and Odometry node.
    If reading from the rosbag, start another node that reads from the topics it publishes to."""
    rclpy.init(args=args)
    filename = "odometry_data.bag"
    rosbag_utils = RosbagUtils(filename=filename)
    if read:
        rosbag_utils.read_bag()
        rclpy.spin(rosbag_utils)  # Keep publishing until shut down
        rosbag_utils.close_bag()
    else:
        odometer = OdometryNode()
        rosbag_utils.write_bag()
        rclpy.spin(odometer)
        rclpy.spin(rosbag_utils)  # Keep subscribing until shut down
        rosbag_utils.close_bag()
        odometer.destroy_node()
    rosbag_utils.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python rosbag_utils.py [read/write]")
        sys.exit(1)

    mode = sys.argv[1]
    if mode == 'read':
        main(read=True)
    else:
        main(read=False)
