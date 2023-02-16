from flask import Flask
import rclpy

from ..abstract_node import AbstractNode


class GUINode(AbstractNode):
    """The node for updating the GUI."""

    app = Flask("Remote Interface")

    @app.route("/")
    def _home(self):
        # Display home page of server
        return "Hello World!"

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)
        # TODO: define web server stuff and whatevers

    def _main(self):
        # TODO: handle opening a web server and publishing stuff to it
        self.app.run()


def main(args=None):
    """Run test gui node"""
    rclpy.init(args=args)
    node = GUINode("GUI_Node")
    node.main()
    node.destroy_node()
