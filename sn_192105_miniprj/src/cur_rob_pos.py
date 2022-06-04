#! /usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates 
from tf import transformations

def get_cur_position(msg):
    ### Get the current position of robot in the X and Y direction ###
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

    print cur_pos_x
    print cur_pos_y
    print yaw_



rospy.init_node("get_cur_position")
#Initialize a subscriber to get the ModelState values from gazebo 
sub=rospy.Subscriber('/gazebo/model_states', ModelStates, get_cur_position) 
rospy.spin()