#!/usr/bin/env python


import math
from math import sin, cos, atan2, atan, sqrt, pi
import rospy
#from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Quaternion
# Brings in the SimpleActionClient
import actionlib
from actionlib_msgs.msg import GoalStatus
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
from tf.transformations import quaternion_from_euler
#rospy.init_node('move_base_sequence')
points_seq = []
points_seq = [ [0, 0],[-9.1, -1.2],[10.7, 10.5], [12.6, -1.9],[18.2, -1.4], [-2, 4.] ]

# Only yaw angle required (no ratotions around x and y axes) in deg:
yaweulerangles_seq = []
i = 0
while(i<6):
  x= points_seq[i][0]
  y=points_seq[i][1]
  # convert quaternion to euler
  if(i<5):
    w = atan2(         # arctan(_del_y/_del_x)
    points_seq[i+1][1]  - y, # del y 
    points_seq[i+1][0]  - x) # del x
  else:
    w = 0
  # pose = [
  #     points_seq[i][0], 
  #     points_seq[i][1], 
  #     #euler_from_quaternion([x, y, 0, w])[2]
  #     w
  #       ]
  yaweulerangles_seq.append(w)
  
  i = i+1
#yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')
#List of goal quaternions:
quat_seq = list()
#List of goal poses:
print(yaweulerangles_seq)
pose_seq = list()

#self.pose_seq = [ [0, 0],[-9.1, -1.2],[10.7, 10.5], [12.6, -1.9],[18.2, -1.4], [-2, 4.] ]goal_cnt = 0
for yawangle in yaweulerangles_seq:
    print(yawangle)
    #Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
    quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
n = 3
print(quat_seq)
# Returns a list of lists [[point1], [point2],...[pointn]]
points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
for point in points:
    #Exploit n variable to cycle in quat_seq
    pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
    n += 1
print(pose_seq)
def movebase_client():
  
       # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        global i 
       # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

       # Creates a new goal with the MoveBaseGoal constructor
        while(len(pose_seq)!=0):
          goal = MoveBaseGoal()
          goal.target_pose.header.frame_id = "map"
          goal.target_pose.header.stamp = rospy.Time.now()
         # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
          goal.target_pose.pose.position.x = pose_seq[0][0]
         # No rotation of the mobile base frame w.r.t. map frame
          goal.target_pose.pose.orientation.w =pose_seq[0][2]
          goal.target_pose.pose.position.y = pose_seq[0][1]
         # No rotation of the mobile base frame w.r.t. map frame
          #goal.target_pose.pose.orientation.w = 1.0
         # Sends the goal to the action server.
          client.send_goal(goal)
         # Waits for the server to finish performing the action.
          wait = client.wait_for_result()
         # If the result doesn't arrive, assume the Server is not available
          if not wait:
              rospy.logerr("Action server not available!")
              rospy.signal_shutdown("Action server not available!")
          else:
          # Result of executing the action
              pose_seq.pop(0)
              print(pose_seq)
              movebase_client()   
                    
# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
            

        
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
