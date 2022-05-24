#!/usr/bin/env python
#coding=UTF-8
import os
import tf
import sys
import copy
import time
import math
import rospy
import numpy as np
import random
import actionlib
import threading
import moveit_commander
import moveit_msgs.msg
import cv2, cv_bridge
import roslib
import multiprocessing
roslib.load_manifest('ur_driver')
roslib.load_manifest('dh_hand_driver')

from copy import deepcopy
from math import pi
from cv_bridge import CvBridge,CvBridgeError
from std_msgs.msg import String
# from __future__ import print_function
from tf import transformations,TransformListener
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from moveit_commander.conversions import pose_to_list
from moveit_commander import MoveItCommanderException
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal
from sensor_msgs.msg import JointState,Image,CameraInfo

from io import *
from ctypes import *
from ur_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from multiprocessing import Process
from trajectory_plan import trajectory_plan


trajectory_plan = trajectory_plan()
UR3_Q1 = [-1.1703794638263147, -1.6704886595355433, -1.4986212889300745, 0.022799968719482422, 1.169772744178772, 4.7115678787231445]
# UR3_Q1为准备点
UR3_Q2 = [-1.1703313032733362, -1.6617491880999964, -1.293955151234762, -0.019654575978414357, 1.1697607040405273, 4.711531639099121]


UR10_Q1 = [-0.2823393980609339, -1.4180730024920862, 0.9614772796630859, -1.717309300099508, -1.4651225248919886, 2.0386080741882324]
ur10_pull_ready = [-0.2822197119342249, -1.5342486540423792, 0.961824893951416, -1.717513386403219, -1.4651826063739222, 2.038572072982788]

class test_paper:

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("test_paper", anonymous=True, disable_signals=True)
        # print rospy.get_namespace()

        # self.client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)
        # self.client.wait_for_server()

        ##########################################################################
        reference_frame = "body_link"

        # 初始化UR3规划组
        ur3_group_name = "ur3"
        self.ur3_arm = moveit_commander.MoveGroupCommander(ur3_group_name, ns="ur3")

        # 当运动规划失败后，允许重新规划
        self.ur3_arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        self.ur3_arm.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.ur3_arm.set_goal_position_tolerance(0.001)
        self.ur3_arm.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.ur3_arm.set_max_acceleration_scaling_factor(0.05)
        self.ur3_arm.set_max_velocity_scaling_factor(0.05)

        # 获取终端link的名称
        self.ur3_end_effector_link = self.ur3_arm.get_end_effector_link()

        ur10_group_name = "ur10"
        self.ur10_arm = moveit_commander.MoveGroupCommander(ur10_group_name, ns="ur10")

        # 当运动规划失败后，允许重新规划
        self.ur10_arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        self.ur10_arm.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.ur10_arm.set_goal_position_tolerance(0.001)
        self.ur10_arm.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.ur10_arm.set_max_acceleration_scaling_factor(0.05)
        self.ur10_arm.set_max_velocity_scaling_factor(0.05)

        # 获取终端link的名称
        self.ur10_end_effector_link = self.ur10_arm.get_end_effector_link()
        # print ur10_end_effector_link
        # rospy.sleep(1.0)
        # print self.ur3_arm.get_current_pose()
        # ur3_dish_pose = PoseStamped()
        # ur3_dish_pose.header.frame_id = reference_frame
        # ur3_dish_pose.header.stamp = rospy.Time.now()
        # ur3_dish_pose.pose.position.x = -0.355989505018
        # ur3_dish_pose.pose.position.y = -0.570190369516
        # ur3_dish_pose.pose.position.z = 0.759730899083
        # ur3_dish_pose.pose.orientation.x = -0.0321627088731
        # ur3_dish_pose.pose.orientation.y = -0.741661084261
        # ur3_dish_pose.pose.orientation.z = 0.669884629718
        # ur3_dish_pose.pose.orientation.w = 0.0126086921934
        #
        # self.ur3_arm.set_joint_value_target(UR3_Q1)
        # self.ur3_arm.go()
        #
        # self.ur3_arm.set_pose_target(ur3_dish_pose, self.ur3_end_effector_link)
        # self.ur3_arm.go()
        # self.ur3_arm.set_joint_value_target(UR3_Q1)
        # self.ur3_arm.go()


        # self.ur10_arm.set_joint_value_target(UR10_Q1)
        # self.ur10_arm.go()

        read_ur10_current_pose = tf.TransformListener()
        read_ur10_current_pose.waitForTransform("/body_link", "/ur10_ee_link", rospy.Time(), rospy.Duration(1.0))

        try:
            (start_trans, start_rot) = read_ur10_current_pose.lookupTransform("/body_link", "/ur10_ee_link",
                                                                              rospy.Time(0))
            print start_trans, start_rot
        except:
            pass

        ur10_first_start_pose = PoseStamped()
        ur10_first_start_pose.header.frame_id = reference_frame
        ur10_first_start_pose.header.stamp = rospy.Time(0)
        ur10_first_start_pose.pose.position.x = start_trans[0]
        ur10_first_start_pose.pose.position.y = start_trans[1]
        ur10_first_start_pose.pose.position.z = start_trans[2]

        ur10_first_start_pose.pose.orientation.x = start_rot[0]
        ur10_first_start_pose.pose.orientation.y = start_rot[1]
        ur10_first_start_pose.pose.orientation.z = start_rot[2]
        ur10_first_start_pose.pose.orientation.w = start_rot[3]

        ur10_first_end_pose = PoseStamped()
        ur10_first_end_pose.header.frame_id = reference_frame
        ur10_first_end_pose.header.stamp = rospy.Time(0)
        ur10_first_end_pose.pose.position.x = start_trans[0]
        ur10_first_end_pose.pose.position.y = start_trans[1]
        ur10_first_end_pose.pose.position.z = start_trans[2] + 0.02

        ur10_first_end_pose.pose.orientation.x = start_rot[0]
        ur10_first_end_pose.pose.orientation.y = start_rot[1]
        ur10_first_end_pose.pose.orientation.z = start_rot[2]
        ur10_first_end_pose.pose.orientation.w = start_rot[3]

        # self.ur10_arm.set_pose_target(ur10_first_end_pose, "ur10_tool0")
        # self.ur10_arm.go()
        ur10_waypoints = []
        ur10_waypoints.append(ur10_first_start_pose)
        ur10_waypoints.append(ur10_first_end_pose)

        plan_ur10 = trajectory_plan.B_curve_plan(self.ur10_arm, ur10_waypoints,2)
        print plan_ur10
        # signal_ur10_ready.send(True)
        self.ur10_arm.execute(plan_ur10)


    ##########################################################################
    # @classmethod
    def move_ur10(self):
        for i in range(9):
            print 2
            print self.ur10_end_effector_link
            # time.sleep(2.0)
            self.ur10_arm.set_joint_value_target(UR10_Q1)
            self.ur10_arm.go()
            self.ur10_arm.set_joint_value_target(ur10_pull_ready)
            self.ur10_arm.go()

            # print self.ur10_arm.get_current_pose(self.ur10_end_effector_link)

    # @classmethod
    def move_ur3(self):
        for i in range(9):
            print 1
            print self.ur10_arm.get_current_pose(self.ur10_end_effector_link)
            print self.ur3_end_effector_link
            # time.sleep(1.0)



            # print self.ur3_arm.get_current_pose(self.ur3_end_effector_link)

        # threading.Lock().release()



    

# def move_ur10():
#     for i in range(9):
#         print 2
#         time.sleep(2.0)
#         # self.ur10_arm.set_joint_value_target(UR10_Q1)
#         # self.ur10_arm.go()
#         # self.ur10_arm.set_joint_value_target(ur10_pull_ready)
#         # self.ur10_arm.go()
#
#         # print self.ur10_arm.get_current_pose(self.ur10_end_effector_link)
#
#
# def move_ur3():
#     for i in range(9):
#         print 1
#         time.sleep(1.0)
        
      
if __name__ == '__main__': 
    try:
        test_paper()

        # function_list = [p.move_ur3, p.move_ur10]
        # pool = multiprocessing.Pool(4)
        # for func in function_list:
        #     pool.apply_async(func)
        #     print "end"
        #
        # pool.close()
        # pool.join()
    except rospy.ROSInterruptException:
        pass