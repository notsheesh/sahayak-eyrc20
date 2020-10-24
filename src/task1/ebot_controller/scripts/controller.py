#!/usr/bin/python2
# -*- coding: utf-8 -*-

"""
authors: 
"""

# imports 
import math
import rospy 
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan 
from tf.transformations import euler_from_quaternion

# parameters !!!
_goal_tolerance = 0.01
_range_max = 10

# for tuning the proportional controller 
_Kp = 1.5
_Ktheta = 4

# Waypoint Generator 
def Waypoints(t):
	"""Generates waypoints along a given continuous and differentiable curve"""
	x = -1 
	y = -1 
	return [x, y]

# Odom callback  
def odom_callback(data):
	global pose
	# convert quaternion to euler
	x  = data.pose.pose.orientation.x;
	y  = data.pose.pose.orientation.y;
	z  = data.pose.pose.orientation.z;
	w  = data.pose.pose.orientation.w;
	pose = [
			data.pose.pose.position.x, 
			data.pose.pose.position.y, 
			euler_from_quaternion([x,y,z,w])[2]
		]

# Laser callback  
def laser_callback(msg):
	global regions
	global _range_max
	_sec = 720//5
	# TODO
	regions = {
		'bright': min(min(msg.ranges[_sec*0 : _sec*1-1]), _range_max),
		'fright': min(min(msg.ranges[_sec*1 : _sec*2-1]), _range_max),
		'front' : min(min(msg.ranges[_sec*2 : _sec*3-1]), _range_max),
		'fleft' : min(min(msg.ranges[_sec*3 : _sec*4-1]), _range_max),
		'bleft' : min(min(msg.ranges[_sec*4 : _sec*5-1]), _range_max),
	}

# Helper functions 
def _getDeviation(current_pose, goal_pose):
	"""Calculate pose deviation """
	del_x = current_pose[0] - goal_pose[0]
	del_y = current_pose[1] - goal_pose[1]
	distance = sqrt(pow(del_x, 2) + pow(del_y, 2))
	theta = math.atan2(goal_pose[1] - pose[1], goal_pose[0] - pose[0]) 
	del_theta = theta - pose[2]
	return [distance, del_theta]

def setTestGoalPose(x = 5, y = 5, theta = 0):
	_goal_pose = Pose()
	_goal_pose.position.x = x
	_goal_pose.position.y = y
	_goal_pose.orientation.z = theta
	goal_pose = [
		_goal_pose.position.x,
		_goal_pose.position.y,
		_goal_pose.orientation.z,
	]
	return goal_pose

# Control loop 
def control_loop():

	# topics 
	_odom = '/odom'
	_scan = '/ebot/laser/scan'
	_vel = '/cmd_vel'

	rospy.init_node('ebot_controller')
	pub = rospy.Publisher(_vel, Twist, queue_size = 10)
	rospy.Subscriber(_scan, LaserScan, laser_callback)
	rospy.Subscriber(_odom, Odometry, odom_callback)
	rate = rospy.Rate(10)

	velocity_msg = Twist()
	velocity_msg.linear.x = 0 
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)

	# Set test goal 
	goal_pose = setTestGoalPose
	while not rospy.is_shutdown():
		##################### go to goal ################################
		while(_getDeviation(pose, goal_pose)[0] > _goal_tolerance):

			print("[{}]Current deviation: ".format(
				rospy.get_time(), 
				_getDeviation(pose, goal_pose)))

			# Proportional controller 
			x = _Kp * _getDeviation(pose, goal_pose)[0]
			z = _Ktheta * _getDeviation(pose, goal_pose)[1]

		#################################################################
		# algorithm for obstacle course goes here 
		# TODO: write algorithm to avoid concave obstacles
		#################################################################

		# log command 
		velocity_msg.linear.x = x
		velocity_msg.angular.z = z
		# for sanity 
		velocity_msg.linear.y  = 0
		velocity_msg.linear.z  = 0
		velocity_msg.angular.x = 0
		velocity_msg.angular.y = 0
		pub.publish(velocity_msg)

		# log iteration 
		print("Controller message pushed at {}".format(rospy.get_time()))

		# zzz for 10hz  
		rate.sleep()

if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass