#!/usr/bin/env python

import rospy
from rtt_new import RRT, Node
#from commands import fcuModes,Controller
import numpy as np
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped,Point
from mavros_msgs.msg import PositionTarget
import time
from sklearn.neighbors import KDTree
import threading


#For Visualization Purpose
from Extra import extra

class Navigator():
    def __init__(self):
        rospy.init_node('controller',anonymous=True)
        self.current_pos = PoseStamped()
        self.end_node = Node(0,0,0)
        self.start_node = Node(0,0,0)
        self.path = []
        self.pub = rospy.Publisher('/mavros/setpoint_raw/local',PositionTarget,queue_size=1)
        self.obstacle_set_mutex = threading.Lock()
        t1 = threading.Thread(target = self.ros_threads)
        t1.start()
        


    #function to check the point cloud
    def pointcloud_check(self,msg):
        x = self.start_node.x + msg[0]
        y = self.start_node.y + msg[1]
        z = self.start_node.z + msg[2]
        if x<=0 or y<=0 or z<=0.1:
            return None
        else:
            hello = (x,y,z)
            return hello    
    


    #function to set target
    def set_target(self,point):
        self.end_node.x = point[0]
        self.end_node.y = point[1]
        self.end_node.z = point[2]
    
    #callback function to set the values in current node 
    def get_current_pos(self,msg):
        self.current_pos.pose.position.x = msg.pose.position.x
        self.current_pos.pose.position.y = msg.pose.position.y
        self.current_pos.pose.position.z = msg.pose.position.z
        

    #function to update the position of the current node
    def update_start_pos(self):
        self.start_node.x = self.current_pos.pose.position.x
        self.start_node.y = self.current_pos.pose.position.y
        self.start_node.z = self.current_pos.pose.position.z
        
    def ros_threads(self):
        print("spawn ros threads")
        rospy.Subscriber("stereo/points2",PointCloud2,self.generate_pointcloud,queue_size=1)
        rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.get_current_pos)
        #self.listen()
        #rospy.spin()

    #function to generate the pointcloud path
    def generate_pointcloud(self, msg):
        map = []

        for p in pc2.read_points(msg, field_names=('x','y','z'),skip_nans=True):
            map.append((p[2],p[0],p[1]))

        acquired = self.obstacle_set_mutex.acquire(True)
        if acquired:
            tree = KDTree(np.array(map))
            
            if len(self.path) != 0:
                self.check_path(tree)
                
            if len(self.path) == 0:
                self.plan(tree)
        else:
            print("lock not acquired")
        self.obstacle_set_mutex.release()            

    
    #plan the path using the kdtree 
    def plan(self, tree):
        print("Hi! i am path planner")
        self.update_start_pos()
        path_temp=[]
        path_fin =  RRT(tree,self.start_node,self.end_node,9,3,2)
        
        path_temp = path_fin.main()
        if len(path_temp)!=0:
            self.path = path_temp
        else: 
            plan(tree)    
        

    def check_path(self,tree):
        print("Hi!, i am checking the path for obstacle")
        time1 = time.time()
        for j in range(0,len(self.path)-1):
            p1 = self.path[j]
            p2 = self.path[j+1]
            count = self.getDistance(p1,p2)/0.05
            dx = (p2.x-p1.x)/count
            dy = (p2.y-p1.y)/count
            dz = (p2.z-p1.z)/count

            for i in range(int(count)):
                point = np.array([(i*dx+p1.x,i*dy+p1.y,i*dz+p1.z)])
                near = tree.query_radius(point,r = 0.5,count_only=True)
                if near[:1] != 0:
                    self.plan(tree)
                    print('path changed')
                    return
                else:
                    continue 

        time2 = time.time()
        print("the time taken is ",time2-time1)
        #self.plot.path_plotting(self.path)

    
    def getDistance(self,p1,p2):
        return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)**0.5

    def publish_path(self):
        point = PositionTarget()
        point.type_mask = int('010111111000', 2)
        point.coordinate_frame = 1
        point.position.x = self.path[0].x
        point.position.y = self.path[0].y
        point.position.z = self.path[0].z
        self.pub.publish(point)
    

    #Main function
    def listen(self):
        print('hello')
        rospy.sleep(5)

        while not(rospy.is_shutdown()) and (self.getDistance(self.path[0],self.end_node) >=0.1):
            
            self.publish_path()
            if (self.getDistance(self.path[0],self.current_pos.pose.position)<=0.2):
                self.path.pop(0)
        
        

if __name__ == "__main__":
    nav = Navigator()
    nav.set_target((8,0,0))
    nav.listen()




