#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from math import sin, cos, tan




# declare all the global variables
_range_max = 10
pose=[]
goal_thresh=0.3

regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}

state_ = 0

state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}

def Waypoints(t):
    x_coordinates=[((2*math.pi*i)/10) for i in range(11)]
    y_coordinates=[t(x_coordinates[i]) for i in range(11)]
    return [x_coordinates,y_coordinates]


def control_loop():
    
    global pose
    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    trajectory = lambda x: 2 * sin(x) * sin(x/2)
    waypoint = Waypoints(trajectory)
    # print(waypoint)

    while not rospy.is_shutdown():
        
        i=0
        m=0
        while i < (len(waypoint[0])):
            if len(pose)>0:
                x1=pose[0]
                y1=pose[1]
                ebot_theta=pose[2]
            
                x2 = waypoint[0][i]
                y2 = waypoint[1][i]
                theta_goal=math.atan2((y2-y1), (x2-x1))
                e_theta=theta_goal-ebot_theta
               # print(pose)
                
                dist=math.sqrt((x2-x1)**2+(y2-y1)**2) 
                velocity_msg.linear.x = 0.2*(dist)
                velocity_msg.angular.z = 0.7*(e_theta)
                print(velocity_msg.linear.x)
                print(velocity_msg.angular.z)
                # velocity_msg.linear.x=1
                pub.publish(velocity_msg)
                if(dist< 0.3):
                    i+=1
        while (True):
            
            
            rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
            msg = Twist()
            if state_ == 0:
                msg = find_wall()
            elif state_ == 1:
                msg = turn_left()
            elif state_ == 2:
                msg = follow_the_wall()
                pass
            else:
                rospy.logerr('Unknown state!')
           
            pub.publish(msg)
            
            rate.sleep()
                    
            if(abs(round(pose[1],3)) <= 0.1 ):
                print("Hello")
                m = m+1
                print(m)
                while(m>=40):
                    

                    x1=pose[0]
                    y1=pose[1]
                    ebot_theta=pose[2]
                
                    x2 = 12
                    y2 = 0
                    theta_goal=math.atan2((y2-y1), (x2-x1))
                    e_theta=theta_goal-ebot_theta
                   # print(pose)
                    
                    dist=math.sqrt((x2-x1)**2+(y2-y1)**2) 
                    velocity_msg.linear.x = 0.2*(dist)
                    velocity_msg.angular.z = 0.7*(e_theta)

                    if(dist < 0.3):
                        print("Reached the goal")
                        velocity_msg.linear.x = 0
                        velocity_msg.angular.z = 0

                    pub.publish(velocity_msg)



            #print("Controller message pushed at {}".format(rospy.get_time()))
            rate.sleep()




def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]




def laser_callback(msg):
    global regions_
    regions_ = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    
    take_action()
    

def change_state(state):
    global state_, state_dict_
    if state is not state_:
        #print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0
    state_description = ''
    
    d = 0.7
    
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 4 - fleft'
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        state_description = 'case 5 - front and fright'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        state_description = 'case 6 - front and fleft'
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 7 - front and fleft and fright'
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        state_description = 'case 8 - fleft and fright'
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = -0.42
    return msg

def turn_left():
    msg = Twist()
    msg.angular.z = 0.45
    return msg

def follow_the_wall():
    global regions_
    
    msg = Twist()
    msg.linear.x = 0.5
    return msg

# def laser_callback(msg):
    # global regions
    # global _range_max
    # _sec = 144

    # # _sec = 720//5
    # regions = {
        # 'bright': min(min(msg.ranges[_sec*0 : _sec*1-1]), _range_max),
        # 'fright': min(min(msg.ranges[_sec*1 : _sec*2-1]), _range_max),
        # 'front' : min(min(msg.ranges[_sec*2 : _sec*3-1]), _range_max),
        # 'fleft' : min(min(msg.ranges[_sec*3 : _sec*4-1]), _range_max),
        # 'bleft' : min(min(msg.ranges[_sec*4 : _sec*5-1]), _range_max),
            # }


if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
