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
from sensor_msgs.msg import Imu

if sys.version_info.major < 3:
    import thread as _thread
else:
    import _thread
def callback(message):
#	print(message)
        x = message.orientation.x
	y = message.orientation.y
	z = message.orientation.z
	w = message.orientation.w
	myString =str(str(x)+","+str(y)+","+str(z)+","+str(w))
	myheading = math.atan2(x*y+w*z,0.5-y*y-z*z)
	myheading = myheading*180/3.142
        my_quat.publish(myheading)
	
        print(myheading)

if __name__ == '__main__':
    try:
	rospy.init_node('IMU_maaz')
        my_quat = rospy.Publisher("heading", Int16 , queue_size=20)
        rospy.Subscriber("/imu/data_raw", Imu , callback)
        # subscribed topic should be a parameter
        rospy.init_node('IMU_maaz')
        rospy.spin()
        # spin is the ROS while LOOP

    except KeyboardInterrupt:
        print "Key-interrupt"
        sys.exit(0)

    sys.exit(0)

