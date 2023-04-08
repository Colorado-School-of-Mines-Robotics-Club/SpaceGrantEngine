from typing import List, Type, Tuple

from rclpy.node import Node, Subscription
from std_msgs.msg import String

from .abstract_node import AbstractNode


class Engine(Node):
    """Highest level abstraction of the SpaceGrantEngine ROS2 package."""

    def __init__(self) -> None:
        super().__init__("engine")
        self._nodes: List[AbstractNode] = []
        self._logger = self.get_logger()

        self._heartbeat_node = Node("HeartbeatSubscriber")
        self._heartbeat_sub = self._heartbeat_node.create_subscription(
            String, "heartbeat", self._heartback_callback, 10
        )

        self._logger.info("Started engine node")

    @property
    def heartbeat(self) -> Tuple[Node, Subscription]:
        """Get the node and subscriber for the heartbeat topic."""
        return self._heartbeat_node, self._heartbeat_sub

    def _heartback_callback(self, msg: String):
        data_str = msg.data.split(":")[1]
        out_str = f"Heartbeat Received: {data_str}"
        self._logger.info(out_str)

    def launch_node(self, node_type: Type[AbstractNode], *args, **kwargs) -> None:
        """Given an AbstractNode, constructs it and launchs the node."""
        node = node_type(*args, **kwargs)
        self._nodes.append(node)
        node.launch()
