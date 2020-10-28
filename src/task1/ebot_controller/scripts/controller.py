#!/usr/bin/python2
# -*- coding: utf-8 -*-

"""
authors: 
"""

# imports 
import rospy 
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from math import sin, cos, atan2, atan, sqrt, pi
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion

# parameters !!!
_goal_tolerance = 0.9
_range_max = 10
_samples = 20
_rate = 100 

# controller gains 
_Kp     = 0.14
_Ktheta = 0.98

# Kp = 0.14, Ktheta = 0.98, samples = 20 worked good as of 28/10 2:23 pm


# sensor data containers  
pose = []
regions = {}

# Waypoint Generator 
def Waypoints(t):
	"""
		generates waypoints along a given 
		continuous and differentiable curve t
	"""
	# que x coordinates 
	xs = [(2 * pi * x)/_samples for x in range(_samples)]

	# mini goal waypoint = [x, y, theta]
	waypoint_buffer = [[x, t(x)] for x in xs]

	return waypoint_buffer

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
	rospy.Rate(_rate).sleep()

# Laser callback  
def laser_callback(msg):
	global regions
	global _range_max

	_sec = 720//5
	regions = {
		'bright': min(min(msg.ranges[_sec*0 : _sec*1-1]), _range_max),
		'fright': min(min(msg.ranges[_sec*1 : _sec*2-1]), _range_max),
		'front' : min(min(msg.ranges[_sec*2 : _sec*3-1]), _range_max),
		'fleft' : min(min(msg.ranges[_sec*3 : _sec*4-1]), _range_max),
		'bleft' : min(min(msg.ranges[_sec*4 : _sec*5-1]), _range_max),
	}

# helper function 
def getDev(_current, _goal):
	""" get deviation between two poses """
	# angle  
	theta = atan2( 				 # arctan(_del_y/_del_x)
		_goal[1]  - _current[1], # del y 
		_goal[0]  - _current[0]) # del x
	_del_theta = theta - _current[2]

	# distance   
	_distance = sqrt(
		pow(_goal[0] - _current[0], 2) 
		+ 
		pow(_goal[1] - _current[1], 2))

	return [_distance, _del_theta]

# control loop 
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

	# task 1.0: traverse the specified curve 
	trajectory 		= lambda x: 2 * sin(x) * sin(x/2)
	waypoint_buffer = Waypoints(trajectory)

	# task 2.0: go to goal base 
	big_goal = [12.0, 0]
	waypoint_buffer.append(big_goal)

	_num_wp = len(waypoint_buffer) # for debugging
	rospy.Rate(1).sleep() # wait for first odom value
	
	while not rospy.is_shutdown():
		while (len(waypoint_buffer)):

			##################### pick first mini goal #######################
			goal_pose = waypoint_buffer.pop(0)

			##################### go to mini goal ############################
			# proportional controller 

			while(getDev(pose, goal_pose)[0] > _goal_tolerance):
				x = _Kp     * getDev(pose, goal_pose)[0] 
				z = _Ktheta * getDev(pose, goal_pose)[1] 

				##############################################################
				# algorithm for obstacle course goes here 
				# TODO: write bug algorithm 
				# if(len(waypoint_buffer) == 1):
					# x = -1
					# z = -1
				##############################################################

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

				if(_num_wp == 1):
					print("[INFO] Trajectory executed, planning final goal")

				# zzz for <_rate>hz  
				rate.sleep()

		# stop & break control loop 
		stop_vel = Twist() 
		pub.publish(stop_vel)
		print("[INFO] Reached final goal!")
		break 

if __name__ == '__main__':
	try:
		control_loop()
	except rospy.ROSInterruptException:
		pass