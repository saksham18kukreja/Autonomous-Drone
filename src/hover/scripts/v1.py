#!/usr/bin/env python

import rospy
from path_planner.rtt import RRT, Node
from commands import fcuModes,Controller
import numpy as np
from octomap_msgs.msg import octomap_msgs, Octomap, OctomapWithPose
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped,Point
import threading 
import time
import multiprocessing


class Navigator():
    def __init__(self):
        manager = multiprocessing.Manager()
        rospy.init_node('hello',anonymous=False)
        self.current_pos = Point()
        self.end_node = Node(0,0,0)
        self.start_node = Node(0,0,0)
        self.path = manager.list()
        self.path_new = PoseStamped()
        self.occupancy_map = []
        self.occupancy_map_new = []
        self.obstacle_set_mutex = threading.Lock()
        #self.path_plan_mutex = threading.lock()
        self.time1 = time.time()  
        # self.nav_command_mutex = threading.Lock()
        t1 = threading.Thread(target=self.ros_thread)
        t1.start()
        self.sp_pub = rospy.Publisher('mavros/setpoint_position/local',PoseStamped,queue_size=1)


    def ros_thread(self):
        
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.get_current_pos)
        #rospy.Subscriber("/camera/depth/points", PointCloud2, self.generate_pointcloud_path) 
        rospy.spin()

    def generate_pointcloud_path(self,msg):
        map = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if p[2]>7 or p[1]<0:
                continue
            else:
                map.append(Node(p[0],p[1],p[2]))
        acquired = self.obstacle_set_mutex.acquire(True)
        if acquired:
            self.set_occupancy_map(map)
            self.obstacle_set_mutex.release()
            return
        else:
            print('lock not acquired')       
        
    def set_occupancy_map(self,msg):
        self.occupancy_map = msg
        
    
    def get_current_pos(self,msg):
        self.current_pos.x = msg.pose.position.x
        self.current_pos.y = msg.pose.position.y
        self.current_pos.z = msg.pose.position.z
        #print(self.current_pos)

    def update_start_pos(self,msg):
        self.start_node.x = abs(msg.x)
        self.start_node.y = abs(msg.y)
        self.start_node.z = abs(msg.z)


    def set_target(self,point):
        self.end_node.x = point[0]
        self.end_node.y = point[1]
        self.end_node.z = point[2]    
    

    def getDistance(self,p1,p2):
        return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)**0.5
    
    def pointcloud_form(self):
        map=[]
        msg = rospy.wait_for_message('/camera/depth/points',PointCloud2)
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if p[2]>7 or p[1]<0:
                continue
            else:
                map.append(Node(p[0],p[1],p[2]))
        return map        
    
    def find_path_around(self):
        while not(rospy.is_shutdown()):
            map = self.pointcloud_form()
            self.update_start_pos(self.current_pos)
            time1 = time.time()
            obj = RRT(map,self.start_node,self.end_node,8,4,3)
            path = obj.main()
            time2 = time.time()
            # if path!=None:
            #     print[(p.x,p.y,p.z)for p in path]
            # else:
            #     print('path not found')
            print('time taken',time2-time1)
            time.sleep(5)    

    def test1(self):
        manager = multiprocessing.Manager()
        map = manager.list()
        task1 = multiprocessing.Process(target=self.find_path_around,args=()).start()
        
            

            

    def main(self):
        self.path = self.find_path_around()
        time1 = rospy.get_time()
        while self.getDistance(self.current_pos,self.end_node)>0.5:
            for next_pose in self.path:
                com = Controller(next_pose)
                com.main()
                if (rospy.get_time()-time1)>5:
                    self.path = self.find_path_around()
                    time1 = rospy.get_time()
                else:
                    continue    
             

if __name__ == "__main__":
    nav = Navigator()
    #t2 = threading.Thread(target=nav.main(),args=())
    nav.set_target((7,0,0))
    nav.test1()
    #t2.start()


