#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool,SetMode,CommandTOL
from mavros_msgs.msg import State
import time

rospy.init_node('hover-takeoff',anonymous=False)
rate = rospy.Rate(10)

# set mode
print('Setting Mode')
rospy.wait_for_service('/mavros/set_mode')
try:
    change_mode = rospy.ServiceProxy('/mavros/set_mode',SetMode)
    response = change_mode(custom_mode = 'OFFBOARD')
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print('SET MODE FAILED')

# Arming
print('arming the drone')
rospy.wait_for_service('/mavros/cmd/arming')
try:
    arming = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    response = arming(value= True)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print('arming failed')    

#Take-off
print('taking off')
rospy.wait_for_service('/mavros/cmd/takeoff')
try:
    takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff',CommandTOL)
    response = takeoff_cl(altitude=10)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print('takeoff failed')

print('hovering')
time.sleep(10)

#landing
print('landing now')
rospy.wait_for_service('/mavros/cmd/land')
try:
    landing = rospy.ServiceProxy('/mavros/cmd/land',CommandTOL)
    response = landing(altitude=10,latitude=0,longitude=0,min_pitch=0,yaw=0)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print('landing failed')

#Disarm
print('Disarming now')
rospy.wait_for_service('/mavros/cmd/arming')
try:
    change_mode = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    response = change_mode(value = False)
    rospy.loginfo(response)
except rospy.ServiceException as e:
    print('disarming failed')            







