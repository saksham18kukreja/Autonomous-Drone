#!/usr/bin/env python
# ROS python API
import rospy

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class fcuModes:
    def __init__(self):
        pass

#    # def setTakeoff(self):
#     	rospy.wait_for_service('mavros/cmd/takeoff')
#     	try:
#     		takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
#     		takeoffService(altitude = 3)
#     	except rospy.ServiceException as e:
#     		print ('Service takeoff call failed:')

    def setArm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arming call failed: %s"%e

    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service disarming call failed: %s"%e

    def setStabilizedMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='STABILIZED')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    def setAltitudeMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='ALTCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    def setPositionMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='POSCTL')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Position Mode could not be set."%e

    def setAutoLandMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException, e:
               print "service set_mode call failed: %s. Autoland Mode could not be set."%e

class Controller:
    def __init__(self):
        self.state = State()
        self.sp = PositionTarget()
        self.sp.type_mask = int('010111111000', 2)
        self.sp.coordinate_frame = 1

        
        self.ALT_SP = 1.0
        self.sp.position.z = self.ALT_SP
        self.STEP_SIZE = 2.0
        self.FENCE_LIMIT = 5.0

        self.local_pos = Point(0.0, 0.0, 1.0)
        self.sp.position.x = 0.0
        self.sp.position.y = 0.0
        


    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z


    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y

    def x_dir(self):
    	self.sp.position.x = self.local_pos.x + 5
    	self.sp.position.y = self.local_pos.y
        

    def neg_x_dir(self):
    	self.sp.position.x = self.local_pos.x - 5
    	self.sp.position.y = self.local_pos.y

    def y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y + 5

    def neg_y_dir(self):
    	self.sp.position.x = self.local_pos.x
    	self.sp.position.y = self.local_pos.y - 5

    def distance(self):
        distance = ((self.sp.position.x-self.local_pos.x)**2 +
                    (self.sp.position.y-self.local_pos.y)**2 + 
                    (self.sp.position.z-self.local_pos.z)**2)**0.5
        return distance            

     


def main():

    rospy.init_node('setpoint_node', anonymous=True)

    modes = fcuModes()

    cnt = Controller()
    cnt.sp_new = []

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
        rate.sleep()
        k = k + 1

    modes.setOffboardMode()
    
    i=1
    loc_dist=1
    dist_error = 0.3
    while not rospy.is_shutdown():
        #cnt.updateSp()
        loc_dist = cnt.distance()
        if loc_dist<=dist_error and i==1:
            cnt.updateSp()
            cnt.x_dir()
            i=i+1
            #rate.sleep()
        elif loc_dist<=dist_error and i==2:
            cnt.updateSp()
            cnt.y_dir()
            i=i+1
            #rate.sleep()
        elif loc_dist<=dist_error and i==3:
            cnt.updateSp()
            cnt.neg_x_dir()
            i = i+1
            #rate.sleep()
        elif loc_dist<=dist_error and i==4:
            cnt.updateSp()
            cnt.neg_y_dir()
            i = 1        
            #rate.sleep()
        sp_pub.publish(cnt.sp)
        print(loc_dist)
        #print(cnt.local_pos)
            

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass