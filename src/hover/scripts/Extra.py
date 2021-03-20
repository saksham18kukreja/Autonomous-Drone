import rospy
import numpy
from visualization_msgs.msg import Marker, MarkerArray

class extra():
    def __init__(self):
        self.path_plan = rospy.Publisher('visualization/marker',MarkerArray,queue_size=10)

    def path_plotting(self,path):
        m_arr = MarkerArray()
        mark_id = 0
        for point in path:
            mk = Marker()
            mk.header.frame_id = 'map'
            mk.action = mk.ADD
            mk.type = mk.CUBE
            mk.id = mark_id
            mark_id+=1
            mk.scale.x = 0.3
            mk.scale.y = 0.3
            mk.scale.z = 0.3 
            mk.color.a = 1
            mk.color.r = 1
            mk.pose.position.x = point.x
            mk.pose.position.y = point.y
            mk.pose.position.z = point.z
            mk.pose.orientation.w = 1
            m_arr.markers.append(mk)
        self.path_plan.publish(m_arr)