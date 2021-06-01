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

   

class Controller:
    
    def __init__(self):        
        self.state = State()
        

	# Callbacks
    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg
        #print(self.state)

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

        

# Main function
    def main(self):
        #error = 0.5
        print('Hi!, i am commander node')
        
        modes = fcuModes()
        rospy.init_node("commander",anonymous=True)
        
        rospy.Subscriber('/mavros/state', State, self.stateCb)
        rospy.sleep(2)

        if not self.state.armed:
            modes.setArm()
            print("drone armed")

        if self.state.mode != "OFFBOARD":
            modes.setOffboardMode()
            print("offboard mode set")

        rospy.spin()



        # while not rospy.is_shutdown():

        #     #print(self.local_pos)
        #     if not self.state.armed:
        #         modes.setArm()
        #         print("drone armed")

            

        #     if self.state.mode != "OFFBOARD": 
        #         modes.setOffboardMode()
        #         #print('mode set to offboard')  

               

if __name__ == "__main__":
    cont = Controller()
    cont.main()   

    


    