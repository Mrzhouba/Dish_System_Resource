#! /usr/bin/env python
#coding=UTF-8
import roslib
roslib.load_manifest('dh_hand_driver')
import rospy
import actionlib

from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal

if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)
    client.wait_for_server()

    goal = ActuateHandGoal(1,50,0)  #motoID,Force,Position
    # Fill in the goal here
    
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))