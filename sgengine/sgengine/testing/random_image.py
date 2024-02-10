import cv2
import rclpy
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException
from sensor_msgs.msg import Image

def main(args=None):
    """
    Main function which exclusively launches the test node
    """
    rclpy.init(args=args)
    random_image = Node("random_image")

    random_image.declare_parameter("topic", rclpy.Parameter.Type.STRING)
    random_image.declare_parameter("width", rclpy.Parameter.Type.INTEGER)
    random_image.declare_parameter("height", rclpy.Parameter.Type.INTEGER)
    topic_name = random_image.get_parameter("topic").get_parameter_value().string_value
    image_publisher = random_image.create_publisher(Image, topic_name, 10)
    try:
        image_width = random_image.get_parameter("width").get_parameter_value().integer_value
        image_height = random_image.get_parameter("height").get_parameter_value().integer_value
    except ParameterUninitializedException as e:
        print("image_width and/or image_height not set, defaulting to 1280,720")
        image_width = 1280
        image_height = 720

    cv_bridge = CvBridge()

    def generate_image():
        generated_image = np.random.randint(0, 255, (image_height, image_width), dtype=np.uint8)
        msg = cv_bridge.cv2_to_imgmsg(generated_image)
        image_publisher.publish(msg)

    random_image.create_timer(1.0 / 5.0, generate_image)

    rclpy.spin(random_image)
    random_image.destroy_node()
    rclpy.shutdown()