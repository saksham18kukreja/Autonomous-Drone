#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,Point
from mavros_msgs.msg import *
from mavros_msgs.srv import *



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
    def __init__(self,current_pos,next_pos):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        # set the flag to use position setpoints and yaw angle
        self.sp.type_mask = int('010111111000', 2)
        # LOCAL_NED
        self.sp.coordinate_frame = 1
        self.local_pos = current_pos
        self.sp.yaw = 0
        

        # initial values for setpoints
        self.sp.position.x = next_pos.x
        self.sp.position.y = next_pos.y
        self.sp.position.z = next_pos.z

	# Callbacks
    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    ## Update setpoint message
    def updateSp(self,msg):
        self.sp.position.x = msg[0]
        self.sp.position.y = msg[1]
        self.sp.position.z = msg[2]

    def finish_task(self):
        self.t1.join()

    def get_current_pos(self,msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
    
    def start_node(self):
        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.get_current_pos)
        #rospy.spin()
    
    def distance(self):
        distance = ((self.sp.position.x-self.local_pos.x)**2 +
                    (self.sp.position.y-self.local_pos.y)**2 + 
                    (self.sp.position.z-self.local_pos.z)**2)**0.5
        return distance     

        

# run the loop till all the setpoints have been covered
    # start node is the first setpoint of the path, end node is the last point
    # if the setpoint is equal to the end point and the error/distance is less than 0.5 , AutoLand on the spot
    # update the position of the setpoint when the distance between the local position and setpoint is 0.5

# Main function
    def main(self):
        error = 0.5
        print('here')
        
        modes = fcuModes()
        rate = rospy.Rate(20.0)
        
        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.get_current_pos)
        sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

        #print(self.local_pos)
        while not self.state.armed:
            modes.setArm()
            #rate.sleep()

        k=0
        while k<10:
            sp_pub.publish(self.sp)
            rate.sleep()
            k = k + 1

        
        modes.setOffboardMode()  

        while self.distance()>=error and not(rospy.is_shutdown()):
            sp_pub.publish(self.sp)   

        #t1.join()

        return None    

    


    