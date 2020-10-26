#!/usr/bin/python2
# imports 
import rospy 
from nav_msgs.msg import Odometry 
from math import sin, cos, atan2, atan, sqrt
from tf.transformations import euler_from_quaternion
import math.pi as PI 

pose = []

# Odom callback  
def odom_callback(data):
	global pose
	# convert quaternion to euler
	x  = data.pose.pose.orientation.x;
	y  = data.pose.pose.orientation.y;
	z  = data.pose.pose.orientation.z;
	w  = data.pose.pose.orientation.w;
	pose = [
			round((data.pose.pose.position.x), 4), 
			round((data.pose.pose.position.y), 4), 
			round((euler_from_quaternion([x, y, z, w])[2]), 4), 		# theta in radians
			round((euler_from_quaternion([x, y, z, w])[2] * 180/PI), 4) # theta in degrees 
		]

def record_loop():

	_odom = '/odom'
	rospy.init_node('pose_recorder')
	rospy.Subscriber(_odom, Odometry,  odom_callback)
	file = open('poses.txt', 'a')
	txt = "[x     , y    , theta, theta_deg]"
	file.write(txt)

	while not rospy.is_shutdown():
		file.write("{}, {}, {}, {}". format(
			pose[0], pose[1], pose[2], pose[3]
			)) 
		rospy.Rate(10).sleep()

	txt = "this is the end of file"
	file.write(txt)
	file.close()

if __name__ == '__main__':
	try:
		record_loop()
	except rospy.ROSInterruptException:
		pass