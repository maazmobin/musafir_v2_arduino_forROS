#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

vl = 0.000
vr = 0.000

def callback(message):
        global vl
	global vr
	vl = message.linear.x
        vr = message.linear.y
##	print(vl+vr)
rospy.Subscriber("twistVelocity", Twist, callback)

WHEELBASE = 0.32 ;                 #m

x = 0.000
y = 0.000
th = 0.000

vx = 0.1
vy = -0.1
vth = 0.1

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10.0)
while not rospy.is_shutdown():
	global vl
	global vr
    	current_time = rospy.Time.now()
	
    	# compute odometry in a typical way given the velocities of the robot
    	dt = (current_time - last_time).to_sec()
	mean_d = ( vl + vr ) * dt
	print(vl + vr)
	delta_x = mean_d * cos(th)
	delta_y = mean_d * sin(th)
   	delta_th = ( ( vr-vl ) * dt ) / WHEELBASE

    	x += delta_x
    	y += delta_y
    	th += delta_th

    	# since all odometry is 6DOF we'll need a quaternion created from yaw
   	odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    	# first, we'll publish the transform over tf
    	odom_broadcaster.sendTransform(
        	(x, y, 0.),
        	odom_quat,
       		current_time,
        	"base_link",
        	"odom"
    	)

    	# next, we'll publish the odometry message over ROS
    	odom = Odometry()
    	odom.header.stamp = current_time
    	odom.header.frame_id = "odom"

    	# set the position
    	odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    	# set the velocity
    	odom.child_frame_id = "base_link"
    	odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    	# publish the message
    	odom_pub.publish(odom)

    	last_time = current_time
    	r.sleep()

