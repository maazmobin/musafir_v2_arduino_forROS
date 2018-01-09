#!/usr/bin/env python
import sys,serial
import threading
import __future__
from serial.tools import list_ports
import math
import time
import tf
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

if sys.version_info.major < 3:
    import thread as _thread
else:
    import _thread

def callback(message):
        v = message.linear.x
        w = message.angular.z
        a = 2*v
	b = w*0.330 #0.33m is wheel base
        VL_desired = (a-b)/0.16	#0.16m is wheel diameter
	VR_desired = (a+b)/0.16
        VL.publish(VL_desired)
        VR.publish(VR_desired)

        print(VL_desired)

if __name__ == '__main__':
    try:
	rospy.init_node('twist2DiffVel')
        VL = rospy.Publisher("lwheel_desired_vel", Int16 , queue_size=2)
	VR = rospy.Publisher("rwheel_desired_vel", Int16 , queue_size=2)
        rospy.Subscriber("/cmd_vel", Twist, callback)
        # subscribed topic should be a parameter
        #rospy.init_node('twist2DiffVel')
        rospy.spin()
        # spin is the ROS while LOOP

    except KeyboardInterrupt:
        print "Key-interrupt"
        sys.exit(0)

    sys.exit(0)
