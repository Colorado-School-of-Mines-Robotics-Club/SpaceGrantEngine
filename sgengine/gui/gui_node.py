from flask import Flask

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
        print("Init")
        pass

    def _main(self):
        # TODO: handle opening a web server and publishing stuff to it
        print("Run")
        self.app.run()
        pass


def main():
    """Run test gui node"""
    print("test")
