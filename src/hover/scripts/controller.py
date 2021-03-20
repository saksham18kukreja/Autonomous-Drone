#!/usr/bin/env python

import rospy
from path_planner.new import RRT, Node
from commands import fcuModes,Controller
import numpy as np
from octomap_msgs.msg import octomap_msgs, Octomap, OctomapWithPose
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped,Point
import time


#For Visualization Purpose
from Extra import extra

class Navigator():
    def __init__(self):
        self.current_pos = Point()
        self.end_node = Node(0,0,0)
        self.start_node = Node(0,0,0)
        self.path = []
        self.occupancy_map = []
        self.path_new = []

        
    # TODO: will use later 
    #function to add point cloud as node to the 3d space
    def pointcloud_callback(self,msg):
        map = []
        for p in pc2.read_points(msg, field_names=('x','y','z'),skip_nans=True):
            point = self.pointcloud_check(p)
            if point==None:
                continue
            else:
                #print(point[1])
                map.append(Node(point[0],point[1],point[2]))        
        return map            
#self.start_node.x +
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
    
    #function to generate occupancy map (to be deleted)
    def generate_map(self,msg):
        map = []
        map.append(Node(msg[0], msg[1], msg[2]))
        return map


    #function to set target
    def set_target(self,point):
        self.end_node.x = point[0]
        self.end_node.y = point[1]
        self.end_node.z = point[2]
    
    #callback function to set the values in current node 
    def get_current_pos(self,msg):
        self.current_pos.x = msg.pose.position.x
        self.current_pos.y = msg.pose.position.y
        self.current_pos.z = msg.pose.position.z
        

    #function to update the position of the current node
    def update_start_pos(self,msg):
        self.start_node.x = abs(msg.x)
        self.start_node.y = abs(msg.y)
        self.start_node.z = abs(msg.z)
        #print(start)
    
    def generate_pointcloud_path(self):
        msg = rospy.wait_for_message('/stereo/points2',PointCloud2)
        self.update_start_pos(self.current_pos)
        self.occupancy_map = self.pointcloud_callback(msg)
        print(np.shape(self.occupancy_map))
        print(self.start_node.x)
        print(self.end_node.x)
        path = self.path_around(self.occupancy_map,self.start_node,self.end_node)
        if path!= None:
            print[(p.x,p.y,p.z) for p in path]

        plot = extra()
        plot.path_plotting(path)    
        return path

    def getDistance(self,p1,p2):
        return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)**0.5

    #Main function
    def listen(self):
        print('hello')
        rospy.init_node('controller',anonymous=True)
        rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.get_current_pos)


    # what i ought to do to make this shit work for surprise obstacles in front of the drone
    # update the start point of the node, call the point cloud data and create the map and path around the obstacle
    # after 5 sec call the point cloud node again to detect any new obstacle
    # call the path finder again to create a new path 
    # how to choose which path to follow
    # if the max distance in this new point cloud is greater than 5 m means that that the new point cloud detects a new obstacle
    # if the above condition holds true then change the path to this new path else continue with old 

        
        
            
        self.path = self.generate_pointcloud_path()
            
            
        # give the path values to the command node
        time1 = rospy.get_time()
        #print(time1)
        
        if self.path != None:
            for next_move in self.path:
                com = Controller(self.current_pos,next_move)
                com.main()
                if int(rospy.get_time()- time1) >=5 and self.getDistance(self.current_pos,self.end_node) >= 0.5:
                    time1 = rospy.get_time()
                    self.path = self.generate_pointcloud_path()
                #com.finish_task()    
        else: 
            pass    
       
                
                
    # function to call the path_planner to calculate the path around the obstacle            
    def path_around(self,occupancy,start,end):
        time0 = time.time()
        obj = RRT(occupancy,start,end,9,3,2)
        path = obj.main()
        time2 = time.time()
        print('time taken is', time2-time0)
        return path

if __name__ == "__main__":
    nav = Navigator()
    nav.set_target((8,0,0))
    nav.listen()
    #nav.set_start_pos()
    #print(nav.start_node)    


