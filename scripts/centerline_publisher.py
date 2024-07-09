#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import pandas as pd

class CenterlineNode(Node):
    def __init__(self):
        super().__init__('centerline_visualization_node')
        self.publisher = self.create_publisher(Marker, '/visualization_marker',10)
        self.timer=self.create_timer(1/60,self.marker_publisher)
        self.load_pts()
        