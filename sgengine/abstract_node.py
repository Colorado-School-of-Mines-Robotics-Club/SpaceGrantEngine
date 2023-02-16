from abc import ABC, abstractmethod
from functools import cached_property
from typing import List, Any, Callable, Dict, Optional

import rclpy
from rclpy.node import Node, Publisher, Subscription
from std_msgs.msg import String


class AbstractNode(ABC, Node):
    """Abstract class for nodes of the sub modules to implement."""

    def __init__(self, name: str) -> None:
        ABC.__init__(self)
        Node.__init__(self, name)
        self._name = name

        # dummy allocations
        self._publishers: Optional[Dict[str, Publisher]] = None
        self._subscribers: Optional[Dict[str, List[Subscription]]] = None
        self._heartbeat_publisher: Optional[Publisher] = None
        self._heartbeat_timer: Optional[Any] = None
        self._heartbeat_counter: int = None
        self._initialized: bool = False

    def init(self) -> None:
        """
        Setups the data structures for using publishers and subscribers through the abstract node interface.

        If this is not called, this node acts as a normal ROS2 node
        """
        # setup local tracking
        self._publishers: Dict[str, Publisher] = [lambda: None]
        self._subscribers: Dict[str, List[Subscription]] = [lambda: []]

        # setup stuff for heartbeat
        self._heartbeat_publisher = self.create_publisher(String, "heartbeat", 10)
        self._heartbeat_timer = self.create_timer(1.0, self._heartbeat_callback)
        self._heartbeat_counter: int = 0

    def _heartbeat_callback(self):
        msg = String()
        msg.data = f"Heartbeat: {self._name}, {self._heartbeat_counter}"
        self._heartbeat_publisher.publish(msg)
        self._heartbeat_counter += 1

    def launch(self) -> None:
        """
        Launch the node.

        Should only be called on a given node after the instance has been created,
        BUT before the .init() call is made.
        """
        pass  # TODO, idk if return LaunchDescription or if we want to just use subprocess

    @cached_property
    def logger(self) -> Any:
        """Return the ROS logger instances."""
        return self.get_logger()

    def _create_publisher(self, topic: str, data: Any, queue_size=10) -> None:
        assert data is not None
        self._publishers[topic] = self.create_publisher(type(data), topic, queue_size)

    def publish(self, topic: str, data: Any) -> None:
        """Publish a given data packet to a given topic."""
        assert self._initialized
        if self._publishers[topic] is None:
            self._create_publisher(topic, data)
        self._publishers[topic].publish(data)

    def _create_subscriber(
        self, topic: str, msg_datatype: Any, callback: Callable, queue_size=10
    ) -> None:
        assert msg_datatype is not None
        self._subscribers[topic].append(
            self.create_subscription(type(msg_datatype), topic, callback, queue_size)
        )

    def subscribe(
        self, topic: str, callback: Callable, msg_datatype: Any = String, queue_size=10
    ) -> None:
        """Add a callback function as a subscriber to a given topic."""
        assert self._initialized
        self._create_subscriber(topic, msg_datatype, callback, queue_size)

    def main(self) -> None:
        """Run the main function (or entry point) into the given Node."""
        self.init()

        # execute the end behavior of the AbstractNode implementations
        self._main()

        rclpy.spin(self)
        self.destroy_node()
        rclpy.shutdown()

    @abstractmethod
    def _main(self) -> None:
        """Implement any publish/subscribe behavior in this method."""
        pass
