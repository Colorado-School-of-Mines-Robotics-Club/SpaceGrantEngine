# pylint: skip-file

from flask import Flask
import rclpy

from rclpy.node import Node


class APINode(Node):
    """The node for running the API."""

    app = Flask("Remote Interface")

    @app.route("/")
    def _home(self):
        # Display home page of server
        return "Hello World!"

    def __init__(self) -> None:
        Node.__init__(self, "api")

        # TODO: spawn new thread to run server
        # self.app.run()
        print("This node has not been implemented!")


def main(args=None):
    """Run test gui node"""
    rclpy.init(args=args)
    api = APINode()
    rclpy.spin(api)
    api.destroy_node()
    rclpy.shutdown()
