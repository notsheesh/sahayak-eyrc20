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
_goal_tolerance = 0.5
_range_max = 10

# for debugging choose sth like 1Hz 
_rate = 100 

# goal pose for debugging 
_goal_x  = 5
_goal_y  = 3
_goal_th = 0

# controller gains 
_Kp     = 0.06
_Ktheta = 0.3

# Kp = 0.06 & Ktheta = 0.3, though sub par, worked as of 25/10/ 2:27 am


# sensor data containers  
pose = []
regions = {}

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
			euler_from_quaternion([x, y, z, w])[2]
		]
	# print("Pose: {}".format(pose)) # for debugging 
	rospy.Rate(_rate).sleep()


# Laser callback  
def laser_callback(msg):
	global regions
	global _range_max

	_sec = 720//5
	# TODO: select range for optimum perception 
	regions = {
		'bright': min(min(msg.ranges[_sec*0 : _sec*1-1]), _range_max),
		'fright': min(min(msg.ranges[_sec*1 : _sec*2-1]), _range_max),
		'front' : min(min(msg.ranges[_sec*2 : _sec*3-1]), _range_max),
		'fleft' : min(min(msg.ranges[_sec*3 : _sec*4-1]), _range_max),
		'bleft' : min(min(msg.ranges[_sec*4 : _sec*5-1]), _range_max),
	}

# Helper functions 
def _getDeviation(current_pose, goal_pose):
	""" get deviation between two poses """

	# distance 
	del_x = goal_pose[0] - current_pose[0]
	del_y = goal_pose[1] - current_pose[1]

	# angle  
	theta = math.atan2( 				# arctan(del_y/del_x)
		goal_pose[1]  - current_pose[1], # del y 
		goal_pose[0]  - current_pose[0]) # del x

	# deviation  
	distance = math.sqrt(pow(del_x, 2) + pow(del_y, 2))
	del_theta = theta - current_pose[2]
	print("Distance: {} | Angle: {}".format(distance, del_theta)) # for debugging 

	return [distance, del_theta]

# For testing
def _setTestGoalPose(x = 2, y = 2, theta = 0):
	_goal_pose = Pose()
	_goal_pose.position   .x = x
	_goal_pose.position   .y = y
	_goal_pose.orientation.z = theta
	goal_pose = [
		_goal_pose.position   .x,
		_goal_pose.position   .y,
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
	pub = rospy.Publisher(_vel,  Twist,     queue_size = 10)
	rospy.Subscriber     (_scan, LaserScan, laser_callback)
	rospy.Subscriber     (_odom, Odometry,  odom_callback)
	rate = rospy.Rate(_rate)

	velocity_msg           = Twist()
	velocity_msg.linear .x = 0 
	velocity_msg.angular.z = 0
	pub.publish(velocity_msg)

	# Set test goal (in params above)
	goal_pose = _setTestGoalPose(_goal_x, _goal_y, _goal_th)

	rospy.Rate(1).sleep() # wait for first odom value
	while not rospy.is_shutdown():
		##################### go to goal ################################
		while(_getDeviation(pose, goal_pose)[0] > _goal_tolerance):

			if(_getDeviation(pose, goal_pose)[0] > 10):
				print("Controller has become unstable. Halting...")
				break

			# print("Pose~: {}".format(pose)) # for debugging 
			# proportional controller 
			x = _Kp     * _getDeviation(pose, goal_pose)[0] 
			z = _Ktheta * _getDeviation(pose, goal_pose)[1] 

		#################################################################
		# algorithm for obstacle course goes here 
		# TODO: write algorithm to avoid concave obstacles
		#################################################################

			# log command 
			velocity_msg.linear .x  = x 
			velocity_msg.angular.z  = z 
			# for sanity 
			velocity_msg.linear .y  = 0
			velocity_msg.linear .z  = 0
			velocity_msg.angular.x  = 0
			velocity_msg.angular.y  = 0
			pub.publish(velocity_msg)

			# log iteration 
			print("Controller message pushed at {}".format(rospy.get_time()))

			# zzz for <_rate>hz  
			rate.sleep()

		# stop & break control loop 
		stop_vel = Twist() 
		pub.publish(stop_vel)
		print("Reached goal")
		break 

if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass