from ..abstract_node import AbstractNode


class OdometryNode(AbstractNode):
    """Node for running visual odometry"""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)

        # TODO: attributes
        pass

    def _main(self):
        # TODO: pubs and subs, other nodes
        pass
