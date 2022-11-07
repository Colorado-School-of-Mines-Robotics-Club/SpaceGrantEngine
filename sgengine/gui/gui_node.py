from ..abstract_node import AbstractNode


class GUINode(AbstractNode):
    """The node for updating the GUI."""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)
        # TODO: define web server stuff and whatevers
        pass

    def _main(self):
        # TODO: handle opening a web server and publishing stuff to it
        pass
