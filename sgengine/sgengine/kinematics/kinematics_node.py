from ..abstract_node import AbstractNode


class KinematicsNode(AbstractNode):
    """Node for handling kinematics"""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)

        # TODO: define general kinematics things
        pass

    def _main(self):
        # TODO: what needs to happen for setting up pubs and subs
        pass
