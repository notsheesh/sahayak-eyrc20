#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
authors: 
"""

# imports 
import math
import rospy 
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import LaserScan 
from tf.transformations import euler_from_quaternion

# Waypoint Generator 
def Waypoints(t):
    x = -1 
    y = -1 
    return [x, y]

# Odom callback  
def odom_callback(data):
    global pose
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
    # break the scan array into 5 parts 
    _sec = 720//5
    regions = {
        'bright': msg.ranges[0      : _sec*1-1],
        'fright': msg.ranges[_sec*1 : _sec*2-1],
        'front' : msg.ranges[_sec*2 : _sec*3-1],
        'fleft' : msg.ranges[_sec*3 : _sec*4-1],
        'bleft' : msg.ranges[_sec*4 : _sec*5-1],
    }

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

    while not rospy.is_shutdown():
        # algorithm for obstacle course goes here 
        x = -1 
        y = -1
        #########################################

        velocity_msg.linear.x = x
        velocity_msg.linear.y = y
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