#!/usr/bin/python2
# -*- coding: utf-8 -*-

"""
authors: 
"""

# imports 
import rospy 
import time
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from math import sin, cos, atan2, atan, sqrt, pi
from geometry_msgs.msg import Twist, Pose
from tf.transformations import euler_from_quaternion

# parameters !!!
_goal_tolerance = 0.3
l_th = 1
_range_max = 10
_samples = 20
_rate = 100 

# controller gains 
_Kp     = 0.38
_Ktheta = 4.0

# Kp = 0.14, Ktheta = 0.98, samples = 20 works good as of 28/10 2:23 pm

# for bug algorithm 
# obstacle repulsion 
f_th = 2.3
fr_th = fl_th = 1.2

# commands 
frwd = 0.6
steer = 0.7
sharp = 0.8

# for super precision 
br_th = bl_th = 2 

# sensor data containers  
pose = []
regions = {}

def Waypoints(t):
	"""
		generates waypoints along a given 
		continuous and differentiable curve t
	"""
	# que x coordinates 
	xs = [(2 * pi * x)/_samples for x in range(_samples)]
	waypoint_buffer = [[x, t(x)] for x in xs]

	# to remove bias after the trajectory 
	waypoint_buffer.append([7, 0])
	
	return waypoint_buffer

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

def checkCollision():
	global regions
	global f_th, fr_th, fl_th 
	# bright = regions['bright']  < br_th
	fright = regions['fright']  < fr_th
	front  = regions['front' ]  < f_th
	fleft  = regions['fleft' ]  < fl_th
	# bleft  = regions['bleft' ]  < bl_th

	print("front: {}".format(regions['front']))

	bleft = False
	bright = False
	check = bright or fright or front or fleft or bleft
	return check

def bugFSM():

	x = 0
	z = 0	

	state = None
	command = None

	# extract laser scan values 
	fleft = regions['fleft']
	front = regions['front']
	fright = regions['fright']

	# motion planning logic
	if front > l_th: 
		if fleft > l_th and fright > l_th:
			state = 'Nothing ahead'
			command = 'Go straight'
			x = frwd
			z = 0

		elif fleft > l_th and fright < l_th:
			state = 'fRight'
			command = 'Turn left'
			x = 0
			z = steer

		elif fleft < l_th and fright > l_th:
			state = 'fLeft'
			command = 'Turn right'
			x = 0
			z = -1 * steer

		else:
			state = 'fRight & fLeft'
			command = 'Narrow path'
			x = 0
			z = sharp

	elif front < l_th:
		if fleft > l_th and fright > l_th :
			state = 'Obstacle ahead'
			command = 'Turn fleft'
			x = 0
			z = steer

		elif fleft > l_th and fright < l_th:
			state = 'fRight'
			command = 'Turn left'
			x = 0
			z = steer

		elif fleft < l_th and fright > l_th:
			state = 'Front & fLeft'
			command = 'Turn right'
			x = 0
			z = -1 * steer

		else:
			state = 'Dead end (F, fR, fL)'
			command = 'Reverse'
			x = -1 * frwd
			z = 0
	else: 
		state = 'Lost'
		command = 'Abort'
		x = -1
		z = -1 

	if(True):
		print("[INFO] Controller: bug fsm")
		# print("[INFO] {}".format(state))
		# print("[CMD] {}".format(command))

	return [x, z]

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
	big_goal = [12.0, 0.0]
	waypoint_buffer.append(big_goal)

	_num_wp = len(waypoint_buffer) # for debugging
	rospy.Rate(1).sleep() 		   # wait for first odom value
	
	while not rospy.is_shutdown():
		while (len(waypoint_buffer)):

			##################### pick first mini goal #######################
			goal_pose = waypoint_buffer.pop(0)

			##################### go to mini goal ############################
			# print("[INFO] Collision?: {}".format(checkCollision()))
			while(getDev(pose, goal_pose)[0] > _goal_tolerance):
				if not checkCollision():
					# proportional controller 
					if(True):
						print("[INFO] Controller: proportional")
					x = _Kp     * getDev(pose, goal_pose)[0] 
					z = _Ktheta * getDev(pose, goal_pose)[1] 

				elif checkCollision(): 
					# finite state machine 	
					global _goal_tolerance
					_goal_tolerance = 0.1
					x, z = bugFSM()

				else: 
					print("[ERR] Invalid")

				velocity_msg.linear .x  = x 
				velocity_msg.angular.z  = z 
				pub.publish(velocity_msg)

			# print("Controller message pushed at {}".format(rospy.get_time()))
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