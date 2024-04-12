import rclpy
from rclpy.node import Node
from sgengine_messages.msg import MoveCommand


class PublisherNode(Node):
    def __init__(self):
        super().__init__("pico_debug_node")
        self.publisher_ = self.create_publisher(MoveCommand, "pico/move_command", 10)
        self.timer = self.create_timer(0.5, self.publish_data)
        self.val = 0.15
        self.delta = 0.1

    def publish_data(self):
        move_command = MoveCommand()
        # Set your data here
        move_command.left = self.val
        move_command.right = self.val
        self.publisher_.publish(move_command)
        self.get_logger().info(
            "Publishing move command: left={}, right={}".format(
                move_command.left, move_command.right
            )
        )
        if self.val >= 1.0:
            self.delta = -0.1
        if self.val <= -1.0:
            self.delta = 0.1
        self.val += self.delta


def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
