#! /usr/bin/env python

import rospy
from goal_publisher.msg import GoalPoint, PointArray
import math
import numpy as np

cur_pos_x = -2.23188453108e-05
cur_pos_y = 1.80422033401e-05

all_goals = []
switch=0

#This function gets a set of points form Point Array and stores them in all_targets array 


def get_goals(msg):
    global all_goals, cur_pos_x, cur_pos_y, switch

    if switch == 0:
        all_goals = msg.goals
        switch +=1
    

    dist_goals= [None]*(len(all_goals))

    for i in range(len(all_goals)):
        dist_goals[i]= (math.sqrt(pow((msg.goals[i].x-cur_pos_x),2) + pow((msg.goals[i].y-cur_pos_y),2)))
        
    
    all_goals = [all_goals for _, all_goals in sorted(zip(dist_goals,all_goals))]
    target=all_goals[0]
    print target
       
    all_goals=all_goals[1:]
    print len(all_goals)

    rospy.loginfo("Outside Loop")


rospy.init_node("get_goals")
sub_all_goals=rospy.Subscriber('/goals', PointArray, get_goals)
rospy.spin()

