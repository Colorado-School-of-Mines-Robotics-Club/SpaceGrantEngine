import rclpy
from sgengine import OakDS2Node as d 

from sgengine import gui

rclpy.init(args=None)


oak = d()


# gui = gui.GUINode("Server")
# gui.main()
# rclpy.spin(gui)