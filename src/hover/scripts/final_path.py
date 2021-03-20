# """ 
# final_path.py is the final program being used for static obstacle avoidance
# the algorithm for the problem is 
# 1. subscribe to the local pose and the final pose required, final target
# 2. run the octomap and register the points where the obstacle is present
# 3. use these points on the grid to generate an obstacle map
# 4. give this obstacle map to the path_planning algorithm
# 5. generate a path around the obstacle
# 6. follow this path
#  """


#essential
import numpy
import time
import rtt
import threading
#import thread

# For ROS
import rospy
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Imu, NavSatFix, PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2

# For Mavros
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode

#For Octomap
from octomap_msgs.msg import Octomap,OctomapWithPose, octomap_msgs

class Navigator(object):
    def __init__(self):
        
        #things initialized during the start of the program
        self.mavros_state = 'OFFBOARD'
        rospy.init_node('navigator_node')
        self.rate = rospy.Rate(5)
        self.path = []
        self.cur_target_pos = None
        self.local_pos = PoseStamped()
        self.obtacle_list = []
        self.current_point_cloud = None
        thread = threading.Thread(target = self.ros_thread)
        thread.start() 

    def set_target(self,target):
        # TODO: implement this function to get the latest target during local planning, 
        # done only til the global planning
        self.cur_target_pos = target
    
    
    def get_latest_target(self):
        return self.cur_target_pos

    def get_current_pos(self,msg):
        self.local_pos = msg.pose.position
        
    def get_obstacle_around(self):
        return self.obtacle_list

    def ros_thread(self):
        self.point_cloud_sub = rospy.Subscriber("/camera/depth/point_cloud", PointCloud, self.point_cloud_callback)
        self.octomap_cells_vis = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.octomap_update_callback)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.get_current_pos)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        
    
    def mavros_state_callback(self,msg):
        self.mavros_state = msg
    
    def point_cloud_callback(self,msg):
        self.current_point_cloud = msg

    def octomap_update_callbackK(self,msg):
        #obstacle_set = []
        for p in pc2.read_points(msg, field_names=('x','y','z'),skip_nans=True):
                self.obtacle_list.append(p)
                
    
    def navigation(self):

        #run the loop till the commands expire
        while self.mavros_state=='OFFBOARD' and not(rospy.is_shutdown()):

            self.relative_pos = (0,0,0)
            end_pos = self.get_latest_target()
            current_pos = self.local_pos

            while current_pos != end_pos and not(rospy.is_shutdown()):
                obtacle_map = self.get_obstacle_around()
                path = RRT(obstacle_map,current_pos,end_pos,10,10,10)
                new_path = path.main()


if __name__ == "__main__":
    nav = Navigator()
    nav.set_target(10,10,10)
    nav.navigation()
                    