#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

global pub, change_state 

def get_laser_scan(msg):
    #360/5=72
    regions_={
        'front' : min(min(msg.ranges[0:17]),min(msg.ranges[342:359]),10),
        'frleft' : min(min(msg.ranges[18:54]), 10),
        'frright' : min(min(msg.ranges[306:341]), 10),
        'left' : min(min(msg.ranges[55:90]), 10),
        'right' : min(min(msg.ranges[270:305]), 10),
    }

    #rospy.loginfo(regions_)
    obj_avoidance(regions_)

def obj_avoidance(regions_):

    global change_state 

    safe_distance =0.5

    if regions_['front']> safe_distance and regions_['frleft'] > safe_distance and regions_['frright']> safe_distance:

        state_description =1
        print ("Going straight")
        #change_state(0)

    else:

        state_description= 0 
        print ("Stop")
        #change_state(2)



rospy.init_node("laser_scan")

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
sub_lase_scan=rospy.Subscriber('/scan', LaserScan, get_laser_scan)

rospy.spin()

#<include file="$(find goal_publisher)/launch/goal_publisher.launch"/>
#<include file="$(find turtlebot3_gazebo)/launch/turtlebot3_empty_world.launch"/>