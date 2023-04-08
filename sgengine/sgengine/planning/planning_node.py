from ..abstract_node import AbstractNode


class PlanningNode(AbstractNode):
    """Node for defining planning functionality of a robot"""

    def __init__(self, name: str, *args, **kwargs) -> None:
        super().__init__(name, *args, **kwargs)

        # TODO: define attributes
        pass

    def _main(self):
        # TODO: define pubs and subs
        pass
