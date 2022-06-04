#! /usr/bin/env python

import rospy
import math 
import time 
from gazebo_msgs.msg import ModelStates 
from geometry_msgs.msg import Twist,Point
from tf import transformations

# robot state variables
cur_pos_x = 0
cur_pos_y = 0
yaw_ = 0
# machine state
state_ = 0
s=0

# goal
des_pos_ = Point()
des_pos_.x=1
des_pos_.y=-1
des_pos_.z=0

# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.3

# publishers
pub = None
twist_msg = Twist()

def fix_yaw():
    global yaw_, pub, yaw_precision_, state_, des_pos_,twist_msg
    
    desired_yaw = math.atan2(des_pos_.y - cur_pos_y, des_pos_.x - cur_pos_x)
    err_yaw = desired_yaw - yaw_
    
    rospy.loginfo("Call 2")

    if math.fabs(err_yaw) > 0.1:
        print 1
        twist_msg.angular.z = -0.1 if err_yaw > 0 else 0.1
    
    time.sleep(0.5)

    pub.publish(twist_msg)
    
    time.sleep(0.1)

    # state change conditions
    if math.fabs(err_yaw) <= 0.1:
        print 2
        state_=1
        print 'Yaw error: [%s]' % err_yaw

def get_cur_position(msg):
    ### Get the current position of robot in the X and Y direction ###
    rospy.loginfo("Call 1")
    cur_pos_x= msg.pose[1].position.x
    cur_pos_y= msg.pose[1].position.y 
    
    #### yaw : Getting the orientation of the robot in the XY plane### 
    
    #Getting Rotation around each axis
    quaternion = (msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w) 
    #Converting from Quaternion to euler form (Required to import transformation library)
    euler = transformations.euler_from_quaternion(quaternion) 
    #Assigning the value of rotation in Z to yaw_ variable
    #roll = euler[0], pitch = euler[1]
    yaw_ = euler[2] 
    fix_yaw()




rospy.init_node("fix_yaw")
rospy.loginfo("Point 1")
#Initialize a subscriber to get the ModelState values from gazebo 
sub=rospy.Subscriber('/gazebo/model_states', ModelStates, get_cur_position) 

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rospy.loginfo("Point 2")


rospy.spin()