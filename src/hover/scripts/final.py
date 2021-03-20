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

class Navigator():
    def __init__(self):
        
        self.current_pos = Point()
        self.end_node = Node(0,0,0)
        self.start_node = Node(0,0,0)
        self.path = []
        self.occupancy_map = []
        self.occupancy_map_new = []
        self.obstacle_set_mutex = threading.Lock()  
        self.nav_command_mutex = threading.Lock()
        t1 = threading.Thread(target=self.ros_thread)
        t1.start()
        
          

    # TODO : MAYBE WILL USE LATER
    # #function to generate occupancy map (to be deleted)
    # def generate_map(self,msg):
    #     map = []
    #     map.append(Node(msg[0], msg[1], msg[2]))
    #     return map


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
        while not(rospy.is_shutdown()):
            map = []    
            print('generating pointclouds')
                #msg = rospy.wait_for_message('/camera/depth/points',PointCloud2)
            msg = self.pointcloud_callback()
            print('generating pointclouds')
            for p in pc2.read_points(msg, field_names=('x','y','z'),skip_nans=True):
                print(p[1])
                    # if p[2]>5 or p[1]<0:
                        continue
                    else:
                        map.append(Node(p[0], p[1], p[2]))
                        self.occupancy_map = map
            time.sleep(5)    
               
    def ros_threads(self):
        self.point_cloud_sub = rospy.Subscriber("/camera/depth/points", PointCloud, self.generate_pointcloud_path)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.get_current_pos)
        
    
    
    def pointcloud_callback(self):
        msg = rospy.wait_for_message('/camera/depth/points',PointCloud2)
        #print('generating pointclouds')
        return msg   
        
    # function to call the path_finder using multiprocess    
    def path_finding_process(self):
        while not(rospy.is_shutdown()):
            self.update_start_pos(self.current_pos)

            #print(self.start_node.x)
            #print(self.end_node.x)
            path = self.path_around(self.occupancy_map,self.start_node,self.end_node)
            print('path found!')
            if path!= None:
                print[(p.x,p.y,p.z)for p in path]
            self.path = path
            time.sleep(5.2)    
        
        #return path    
        
    # function to call the path_planner to calculate the path around the obstacle            
    def path_around(self,occupancy,start,end):
        obj = RRT(occupancy,start,end,8,6,3)
        path = obj.main()
        return path   
    
    # funcion to call the commander nodes , input is the path made by the path_finder
    def piece_of_shit_fly(self):
        while not(rospy.is_shutdown()):
            print(self.path)
            if self.path != None:
                for next_move in self.path:
                    print('i am flying')
                    com = Controller(self.current_pos,next_move)
                    com.main()
            else: 
                pass    

    def getDistance(self,p1,p2):
        return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)**0.5

    
    # what ought to be done to correct this shit to work with multiprocessing 
    # create three processes, one each for: building occupancy map, generating path from this map, controls to follow the path
    # the first process will run every 5 secs, the path finder secos process will run after the first process only
    # after the second process the path values are changed and given to the third process
    # Hope this shit will work!
    # first let's have dinner
    
    #Main function
    def listen(self):
        print('hello')
        rospy.init_node('controller',anonymous=True)
        rospy.Subscriber('/mavros/local_position/pose',PoseStamped,self.get_current_pos)
        

        # Creating Multiple Process for different tasks
        build_map = threading.Thread(target=self.generate_pointcloud_path,args=())
        find_path = threading.Thread(target=self.path_finding_process,args=())
        fly_please = threading.Thread(target=self.piece_of_shit_fly,args=())
        
      

        # main loop to generate point cloud and start the path finding node
        k=0
        while not(rospy.is_shutdown()):
            map = []
            print('generating pointclouds')
            msg = self.pointcloud_callback()
            
            for p in pc2.read_points(msg, field_names=('x','y','z'),skip_nans=True):
                if p[2]>5 or p[1]<0:
                    continue
                else:
                    map.append(Node(p[0], p[1], p[2]))
                    self.occupancy_map = map
            if k==0:
                find_path.start()
                #find_path.join()
                fly_please.start()
                fly_please.join()
                k=1        
            print('pointcloud added')
            time.sleep(5)    



        print('finish')

if __name__ == "__main__":
    nav = Navigator()
    nav.set_target((7,0,0))
    nav.listen()
    #nav.set_start_pos()
    #print(nav.start_node)    


