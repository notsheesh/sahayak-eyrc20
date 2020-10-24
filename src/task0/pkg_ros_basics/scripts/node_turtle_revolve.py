#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
import time
from std_srvs.srv import Empty
from turtlesim.msg import Pose

yaw =  0
count = 0 
flag = 0 

def poseCallback(pose_message):
    global  a 
    global  yaw ,flag
    global count , test
    
    if(count == 0):
        a = round(pose_message.theta,1)
        time.sleep(1.5)
        
      
    test =  a 
    count += 1 
    yaw = round(pose_message.theta,1)
    if(yaw == test):
        time.sleep(0.001)
        flag = count

   
def round1():
    loop_rate =  rospy.Rate(10)
    velocity_message = Twist()
    while True:

        velocity_message.linear.x = 2.0
        velocity_message.angular.z = 1.0

        velocity_publisher.publish(velocity_message)
        rospy.loginfo("Moving in a circle")

        if(abs(yaw) == abs(test) and count>flag+10):
            rospy.loginfo("goal reached")
            time.sleep(0.00001)
            break
    
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)
    





if __name__ == '__main__':
    try:
        
        
        rospy.init_node('turtle_rotate', anonymous=True)
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist , queue_size=10)

        position_topic  = "/turtle1/pose"
        pose_subcriber  = rospy.Subscriber(position_topic , Pose , poseCallback)
        
        
        time.sleep(2)
        
        

        
        round1()
        
    except rospy.ROSInterruptException:
        pass