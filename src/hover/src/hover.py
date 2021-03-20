#!/usr/bin/env python
# ROS python API
import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *

class fcuModes:
    def __init__(self):
        pass

#     def setTakeoff(self):
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

    # def setDisarm(self):
    #     rospy.wait_for_service('mavros/cmd/arming')
    #     try:
    #         armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
    #         armService(False)
    #     except rospy.ServiceException, e:
    #         print "Service disarming call failed: %s"%e

    # def setStabilizedMode(self):
    #     rospy.wait_for_service('mavros/set_mode')
    #     try:
    #         flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
    #         flightModeService(custom_mode='STABILIZED')
    #     except rospy.ServiceException, e:
    #         print "service set_mode call failed: %s. Stabilized Mode could not be set."%e

    def setOffboardMode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. Offboard Mode could not be set."%e

    # def setAltitudeMode(self):
    #     rospy.wait_for_service('mavros/set_mode')
    #     try:
    #         flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
    #         flightModeService(custom_mode='ALTCTL')
    #     except rospy.ServiceException, e:
    #         print "service set_mode call failed: %s. Altitude Mode could not be set."%e

    # def setPositionMode(self):
    #     rospy.wait_for_service('mavros/set_mode')
    #     try:
    #         flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
    #         flightModeService(custom_mode='POSCTL')
    #     except rospy.ServiceException, e:
    #         print "service set_mode call failed: %s. Position Mode could not be set."%e

    # def setAutoLandMode(self):
    #     rospy.wait_for_service('mavros/set_mode')
    #     try:
    #         flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
    #         flightModeService(custom_mode='AUTO.LAND')
    #     except rospy.ServiceException, e:
    #            print "service set_mode call failed: %s. Autoland Mode could not be set."%e

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
        self.ALT_SP = 3.0
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
    def updateSp(self):
        self.sp.position.x = self.local_pos.x
        self.sp.position.y = self.local_pos.y
    
    def update_point(self,msg):
        print('hello')
        self.sp.position.x = msg[0]
        self.sp.position.y = msg[1]
        self.sp.position.z = msg[2]
        

    def error(self,p1,p2):
        return ((p1.x-p2.x)**2 + (p1.y-p2.y)**2 + (p1.z-p2.z)**2)**0.5

# Main function
def main():

    rospy.init_node('setpoint_node', anonymous=True)

    modes = fcuModes()

    cnt = Controller()

    rate = rospy.Rate(20.0)

    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)    
    sp_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    path = [(0,0,1),(0,0,1.5),(0,0,2),(0,0,2.5),(0,0,3),(0,0,3.5),(0,0,4),(0,0,4.5),(0,0,5),(0,0,5.5),(0,0,6),(0,0,5.5),(0,0,5),(0,0,4.5),(0,0,4),(0,0,3.5),(0,0,3),(0,0,2.5),(0,0,2)]

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

    i=0
    while not rospy.is_shutdown():
    	#cnt.updateSp()
        #rate.sleep()
    	sp_pub.publish(cnt.sp)
        #print(cnt.sp.position.z)
        #print(err)
        rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
