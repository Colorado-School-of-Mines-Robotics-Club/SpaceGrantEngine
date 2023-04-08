import rclpy
from rclpy.node import Node


class TestNode(Node):
    """Node for testing ros2 functions"""

    def __init__(self):
        super().__init__("my_node_name")
        self.get_logger().info("This node is a test")


def main(args=None):
    """Test main function for ensuring ros2 works"""

    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
