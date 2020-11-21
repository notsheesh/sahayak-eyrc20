#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
team id: 1071
authors: preet, shreesh, gokul, abhiroop 
reference: fiorellasibona
"""
import rospy
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


# wait time
_WAIT_ = 3.0

# set waypoints 
waypoints = [
	# -1.0, 0.0, 0.0
	-9.1, -1.2, 0, 
	10.7, 10.5, 0, 
	12.6, -1.9, 0, 
	18.2, -1.4, 0, 
	-2.0,  4.0, 0
		]
wp_yaws = [30, -60, 90, 0, -60]

class MoveBaseWP():

    def __init__(self):
    	''' setup ''' 
    	global waypoints
    	global wp_yaws
    	global _WAIT_

    	# init vars 
        rospy.init_node('move_base_sequence')

        quat_seq = list()
        self.pose_seq = list()
        self.goal_count = 0
        n = 3

        # DELETE THESE COMMENTS IF EVERYTHING WORKS FINE 
        # waypoints = [
        # 				-9.1, -1.2, 0, 
        # 				10.7, 10.5, 0, 
        # 				12.6, -1.9, 0, 
        # 				18.2, -1.4, 0, 
        # 				-2.0,  4.0, 0
        # 				]
        # wp_yaws = [30, -60, 90, 0, -60]
        # DELETE THESE COMMENTS IF EVERYTHING WORKS FINE 

        # yaw euler -> quat 
        for wp_yaw in wp_yaws:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, wp_yaw * math.pi / 180, axes='sxyz'))))

        # coordinates -> poses 
        points = [waypoints[i : i + n] for i in range(0, len(waypoints), n)]

        for point in points:
            self.pose_seq.append(Pose(Point(*point), quat_seq[n - 3]))
            n += 1

        # move base handler 
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # for debugging
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        # start 
        self.movebase_client()

    def active_callback(self):
    	''' for progress tracking '''
        rospy.loginfo("Goal pose " + str(self.goal_count + 1) + " is now being processed by the Action Server...")

    def feedback_callback(self, feedback):
    	''' for progress tracking '''
        rospy.loginfo("Feedback for goal pose " + str(self.goal_count + 1) + " received")

    def done_callback(self, status, result):
    	''' reached waypoint '''
        self.goal_count += 1

        # for brevity 
        gcnt = self.goal_count
        sgcnt = str(gcnt)

        # check waypoint status
        if status == 2:
            rospy.loginfo("Goal pose " + sgcnt + " received a cancel request after it started executing, completed execution!")

        if status == 3:

            rospy.loginfo("Goal pose " + sgcnt + " reached") 
            
            if gcnt < len(self.pose_seq):
            
                # set next goal
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[gcnt]

                # for debugging | loginfo 
                rospy.loginfo("Sending goal pose " + str(gcnt + 1) + " to Action Server")
                rospy.loginfo(str(self.pose_seq[gcnt]))

                # send goal
                self.client.send_goal(
                	next_goal, 
                	self.done_callback, 
                	self.active_callback, 
                	self.feedback_callback
                	) 

            else:
                rospy.loginfo        ("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return

        if status == 4:
            rospy.loginfo        ("Goal pose " + sgcnt + " was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose " + sgcnt + " aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo        ("Goal pose " + sgcnt + " has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose " + sgcnt + " rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose " + sgcnt + " received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
    	'''  ''' 
    	# for brevity 
    	gcnt = self.goal_count

    	# new goal
        goal = MoveBaseGoal()

        # goal meta info 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now() 
        goal.target_pose.pose            = self.pose_seq[gcnt]
        
        # for debugging 
        rospy.loginfo("Sending goal pose " + str(gcnt + 1) + " to Action Server")
        rospy.loginfo(str(self.pose_seq[gcnt]))
       
       	# initiate 

       	global _WAIT_
        rospy.Rate(1.0 / _WAIT_).sleep()

        self.client.send_goal(
        	goal, 
        	self.done_callback, 
        	self.active_callback, 
        	self.feedback_callback
        	)

        rospy.spin()

if __name__ == '__main__':

    try:
        MoveBaseWP()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

