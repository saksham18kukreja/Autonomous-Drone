#!/usr/bin/env python

from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped,Point

class PointCld():
    def __init__(self):
        self.current_pos = Point()
        