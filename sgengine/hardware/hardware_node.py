from ..abstract_node import AbstractNode


class HardwareNode(AbstractNode):
    """Node for communicating with hardware"""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)

        # TODO: define hardware connections and such
        pass

    def _main(self):
        # TODO: define what to do on startup, i.e. do we need to open connections
        pass
