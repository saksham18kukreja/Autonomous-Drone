#!/usr/bin/env python

#give a maze to the A* algorithm with obstacles added 
#return a path around the obstacles
# plot the path along with the obstacles on a graph using matplot
#use this path to set waypoints for the drone to move around the obstacle
##in the while loop i<=len(path)
##auto land on the last node (goal node)
#write a main function to call and link all the other functions

import rospy
from geometry_msgs.msg import PoseStamped,Point
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import time

# A* algorithm copied from stand-alone algorithm
class Node():

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self,other):
        return self.position == other.position    

def astar(maze,start,end):

    start_node = Node(None,start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None,end)
    end_node.g = end_node.h = end_node.f = 0


    open_list = []
    closed_list = []
    
    open_list.append(start_node)

    while len(open_list) > 0 :

        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f :
                current_node = item
                current_index = index

        open_list.pop(current_index)
        closed_list.append(current_node)
        #print(closed_list[-1].position)

        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  
        

        children = []
        for new_child in [(1,0),(0,-1),(-1,0),(0,1),(1,1),(1,-1),(-1,-1),(-1,1)]:
            #(1,1),(1,-1),(-1,-1),(-1,1)

            node_position = (current_node.position[0] + new_child[0], current_node.position[1] + new_child[1])

            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue 

            new_node = Node(current_node,node_position)

            children.append(new_node)

        for child in children:

            if len([closed_child for closed_child in closed_list if child == closed_child]) > 0:
                continue

            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0])**2 + (child.position[1] - end_node.position[1])**2)
            child.f = child.g + child.h

            if len([open_node for open_node in open_list if child == open_node and child.g > open_node.g]) > 0:
                continue

            #print(child.position)
            open_list.append(child)    

#subplot added with dimensions 10X10
def plot_path(path):
    fig, ax = plt.subplots()
    ax.set(xlim = (0,10), ylim = (0,10))
    plt.grid(True)
    plt.xticks(np.arange(0,11,1))
    plt.yticks(np.arange(0,11,1))
    #add the obstacles with coordinates
    rect1 = Rectangle((6.5,2.5),1,1)
    rect2 = Rectangle((3.5,3.5),1,1)
    rect3 = Rectangle((6.5,6.5),1,1)
    rect4 = Rectangle((1.5,7.5),1,1)
    rect_fin = [rect1,rect2,rect3,rect4]
    for rect in rect_fin:    
        ax.add_patch(rect)
    #plot the path
    x_point = [x[0] for x in path]
    y_point = [x[1] for x in path]
    ax.plot(x_point,y_point,'red')
    plt.show()

#set waypoint using the path and write the nodes for ROS

class fcuModes:

    def __init__(self):
        pass

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1

        # We will fly at a fixed altitude for now
        # Altitude setpoint, [meters]
        self.ALT_SP = 0.45
        # update the setpoint message with the required altitude
        self.sp.position.z = self.ALT_SP
        # Step size for position update
        self.STEP_SIZE = 2.0
		# Fence. We will assume a square fence for now
        self.FENCE_LIMIT = 5.0

        # A Message for the current local position of the drone
        self.local_pos = Point(0.0, 0.0, 3.0)

        # initial values for setpoints
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0

        # speed of the drone is set using MPC_XY_CRUISE parameter in MAVLink
        # using QGroundControl. By default it is 5 m/s.

	# Callbacks

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self,msg):
        self.sp.position.x = msg[0]
        self.sp.position.y = msg[1]

    def distance(self):
        distance = ((self.sp.position.x-self.local_pos.x)**2 +
                    (self.sp.position.y-self.local_pos.y)**2 + 
                    (self.sp.position.z-self.local_pos.z)**2)**0.5
        return distance     



# Main function
def main(maze,start,end):
    t1 = time.time()
    path = astar(maze,start,end)
    t2 = time.time()
    print(path,t2-t1)
    plot_path(path)
    
    rospy.init_node('setpoint_node', anonymous=True)

    modes = fcuModes()

    cnt = Controller()

    rate = rospy.Rate(20.0)

    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


    while not cnt.state.armed:
        modes.setArm()
        rate.sleep()

    
    k=0
    while k<10:
        sp_pub.publish(cnt.sp)
        #print(cnt.sp)
        rate.sleep()
        k = k + 1

    
    modes.setOffboardMode()

# run the loop till all the setpoints have been covered
# start node is the first setpoint of the path, end node is the last point
# if the setpoint is equal to the end point and the error/distance is less than 0.5 , AutoLand on the spot
# update the position of the setpoint when the distance between the local position and setpoint is 0.5


# Main loop for setpoints
    error = 0.5
    loc_dist = 1
    i=0
    while not rospy.is_shutdown():

        while i < len(path):
            loc_dist = cnt.distance()

            #if i == len(path)-1 and loc_dist <= error:
                #modes.setAutoLandMode()
            
            if loc_dist <= error and i != len(path)-1:
                i = i+1
                cnt.updateSp(path[i])
                

            print(cnt.sp.position)
            sp_pub.publish(cnt.sp) 
            #print(loc_dist) 

# Main Method

if __name__ == "__main__":

    maze = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 1, 0],
            [0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 1, 0, 0, 1, 1, 0, 0],
            [0, 0, 1, 1, 0, 0, 1, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]

    start = (0,0)
    end = (6,0)

    main(maze,start,end)




                





            

    	
