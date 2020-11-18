#!/usr/bin/env python
# license removed for brevity
from math import sin, cos, atan2, atan, sqrt, pi
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Quaternion
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
waypoint = []
waypoint = [[0,0],[-9.1, -1.2],[10.7, 10.5], [12.6, -1.9],[18.2, -1.4], [-2, 4.0]]
reach_way = []

i = 0
while(i<6):
  x= waypoint[i][0]
  y=waypoint[i][1]
  # convert quaternion to euler
  if(i<5):
    w = atan2(         # arctan(_del_y/_del_x)
    waypoint[i+1][1]  - y, # del y 
    waypoint[i+1][0]  - x) # del x
  else:
    w = 0
  pose = [
      waypoint[i][0], 
      waypoint[i][1], 
      euler_from_quaternion([x, y, 0, w])[2]
      
        ]
  reach_way.append(pose)
  i = i+1

print(reach_way)
def movebase_client():
  
       # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        global i 
       # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

       # Creates a new goal with the MoveBaseGoal constructor
        while(len(reach_way)!=0):
          goal = MoveBaseGoal()
          goal.target_pose.header.frame_id = "map"
          goal.target_pose.header.stamp = rospy.Time.now()
         # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
          goal.target_pose.pose.position.x = reach_way[0][0]
         # No rotation of the mobile base frame w.r.t. map frame
          goal.target_pose.pose.orientation.w =reach_way[0][2]
          goal.target_pose.pose.position.y = reach_way[0][1]
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
              reach_way.pop(0)
              print(reach_way)
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
