# pylint: skip-file

import sys
import time
import rclpy

# from std_msgs.msg import String
from sgengine_messages.msg import TwoFloat
from rclpy.node import Node
f


class OdometryNode(Node):
    """Node for running visual odometry"""

    def __init__(self) -> None:
        Node.__init__(self, "odometer")

        self._publisher = self.create_publisher(, "odometer/tvec", 10)

        # TODO: attributes
        pass

    def _main(self):
        # TODO: pubs and subs, other nodes
        pass
