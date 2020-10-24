#!/usr/bin/env python
"""
Python code to make the turtle move in a circular path and make it stop after 1 round

Logic:
We have used the pose_message.theta to complete this task 
1. Subscribe to the "/turtle1/pose" topic and extract the theta from message_pose
2. Recorded the very first theta value of the turtle and store it in a variable
3. Using a Callback we are continuously taking the pose values from the turtle1/pose
4. Check if the pose_message.theta values are the same , if yes then stop the turtle
5. If no, continue publishing message to the /turtle1/cmd_vel topic

Authors : Gokul P, Preet, ABhiroop, Shreesh.
eYRC team ID: 1071

"""
import rospy
from geometry_msgs.msg import Twist
import math
import time
from std_srvs.srv import Empty
from turtlesim.msg import Pose

yaw = 0
count = 0
flag = 0
# Callback function for continuously getting the pose values of the turtle from the pose topic /turtle1/pose.


def poseCallback(pose_message):
    global a
    global yaw, flag
    global count, test

# Using a count variable for counting the number of times the pose values were updated using the callback,
# where the first count value = 0 is the original start position of the turtle stored in variable "a"
    if(count == 0):
        a = round(pose_message.theta, 1)
        time.sleep(1.5)

# Storing the value of variable "a" to "test" to use it locally
    test = a
    count += 1

# Storing the updated values of yaw from the callback
    yaw = round(pose_message.theta, 1)
    if(yaw == test):
        time.sleep(0.001)
        flag = count


def round1():
    loop_rate = rospy.Rate(10)
    velocity_message = Twist()
    # To move the robot in a circular motion with linear velocity: 1m/s and angular velocity: 1 rad/s and publish velocity message to the /turtle1/cmd_vel topic.
    while True:

        velocity_message.linear.x = 1.0
        velocity_message.angular.z = 1.0

        velocity_publisher.publish(velocity_message)
        rospy.loginfo("Moving in a circle")

    # If yaw equals to test , thus the robot has again reached its original position and thus breaking the loop
        if(abs(yaw) == abs(test) and count > flag+10):
            rospy.loginfo("goal reached")
            time.sleep(0.00001)
            break

    # Once the loop is broken , publish  velocity_message to the /turtle1/cmd_vel topic to stop the turtle
    velocity_message.linear.x = 0.0
    velocity_message.angular.z = 0.0
    velocity_publisher.publish(velocity_message)


if __name__ == '__main__':
    try:

        # Starting a ROS node named 'turtle_rotate'
        rospy.init_node('turtle_rotate', anonymous=True)

        # Defining a velocity publisher
        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(
            cmd_vel_topic, Twist, queue_size=10)

        # Defining a position subscriber
        position_topic = "/turtle1/pose"
        pose_subcriber = rospy.Subscriber(position_topic, Pose, poseCallback)

        time.sleep(2)

        # Calling function round1 where velocity messages will be published to the topic /turtle1/cmd_vel for moving in a circular path
        round1()

    except rospy.ROSInterruptException:
        pass
    # P.S : It will take almost a second for the turtle to start after doing rosbag play because of the time.sleep() added to the callback function.
