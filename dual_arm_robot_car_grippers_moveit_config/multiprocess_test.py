#!/usr/bin/env python
#coding=UTF-8
import rospy
import time
import numpy as np
import random
import actionlib
import threading
import moveit_commander
import moveit_msgs.msg
import cv2, cv_bridge
import roslib
import multiprocessing
from trajectory_msgs.msg import *
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
UR3_Q1 = [-1.1703794638263147, -1.6704886595355433, -1.4986212889300745, 0.022799968719482422, 1.169772744178772, 4.7115678787231445]
# UR3_Q1为准备点
UR3_Q2 = [-1.1703313032733362, -1.6617491880999964, -1.293955151234762, -0.019654575978414357, 1.1697607040405273, 4.711531639099121]

ur3_air = [-0.014509979878560841, -1.7416704336749476, -1.3646052519427698, 0.047530174255371094, 2.2125096321105957, 4.734982967376709]

UR10_Q1 = [-0.2823393980609339, -1.4180730024920862, 0.9614772796630859, -1.717309300099508, -1.4651225248919886, 2.0386080741882324]
ur10_pull_ready = [-0.2822197119342249, -1.5342486540423792, 0.961824893951416, -1.717513386403219, -1.4651826063739222, 2.038572072982788]

ur10_air_ready = [-0.690136734639303, -1.7547085920916956, 1.82218599319458, -2.638759438191549, -2.236163918172018, 1.3027009963989258]
ur10_air_pull = [-0.6909988562213343, -1.7548044363604944, 1.8054428100585938, -2.727982823048727, -2.2320316473590296, 3.0234971046447754]
class hhhh():
    def __init__(self):
        conn3, conn10 = multiprocessing.Pipe(True)
        move_3 = multiprocessing.Process(target=self.move_ur3, args=(conn3,))
        move_10 = multiprocessing.Process(target=self.move_ur10, args=(conn10,))

        move_3.start()
        move_10.start()
        move_3.join()
        move_10.join()

    def move_ur10(self, conn10):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("test_paper_1", anonymous=True, disable_signals=True)

        reference_frame = "body_link"
        ur10_group_name = "ur10_with_gripper"
        self.ur10_arm = moveit_commander.MoveGroupCommander(ur10_group_name, ns="ur10")

        # 当运动规划失败后，允许重新规划
        self.ur10_arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        self.ur10_arm.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.ur10_arm.set_goal_position_tolerance(0.001)
        self.ur10_arm.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.ur10_arm.set_max_acceleration_scaling_factor(0.1)
        self.ur10_arm.set_max_velocity_scaling_factor(0.1)

        # 获取终端link的名称
        self.ur10_end_effector_link = self.ur10_arm.get_end_effector_link()

        self.ur10_arm.set_joint_value_target(UR10_Q1)
        self.ur10_arm.go()
        self.ur10_arm.set_joint_value_target(ur10_air_ready)
        self.ur10_arm.go()

        while(1):
            x = conn10.recv()
            print x
            if x==True:
                self.ur10_arm.set_joint_value_target(ur10_air_pull)
                ur10_arrive = self.ur10_arm.go()
                if ur10_arrive==True:
                    dish_pull = True
                    conn10.send(dish_pull)
                    self.ur10_arm.set_joint_value_target(UR10_Q1)
                    self.ur10_arm.go()
                    break



        # rospy.spin()
            # print self.ur10_arm.get_current_pose(self.ur10_end_effector_link)

    # @classmethod
    def move_ur3(self, conn3):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("test_paper", anonymous=True, disable_signals=True)
        reference_frame = "body_link"
        num = 0
        # 初始化UR3规划组
        ur3_group_name = "ur3_with_gripper"
        self.ur3_arm = moveit_commander.MoveGroupCommander(ur3_group_name, ns="ur3")
        # 当运动规划失败后，允许重新规划
        self.ur3_arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        self.ur3_arm.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.ur3_arm.set_goal_position_tolerance(0.001)
        self.ur3_arm.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.ur3_arm.set_max_acceleration_scaling_factor(0.1)
        self.ur3_arm.set_max_velocity_scaling_factor(0.1)

        # 获取终端link的名称
        self.ur3_end_effector_link = self.ur3_arm.get_end_effector_link()

        self.ur3_arm.set_joint_value_target(UR3_Q1)
        self.ur3_arm.go()
        self.ur3_arm.set_joint_value_target(ur3_air)
        self.ur3_arm.go()
        bowl_ready = True
        conn3.send(bowl_ready)
        while(1):
            y = conn3.recv()
            self.ur3_arm.set_joint_value_target(UR3_Q1)
            self.ur3_arm.go()
            break
        # for i in range(9):
        #
        #     num += 1
        #     # print self.ur3_arm.get_current_pose()
        #     # print self.ur3_end_effector_link
        #     # time.sleep(1.0)
        #
        #     self.ur3_arm.set_joint_value_target(UR3_Q1)
        #     self.ur3_arm.go()
        #     self.ur3_arm.set_joint_value_target(UR3_Q2)
        #     self.ur3_arm.go(wait=True)
        #     conn3.send(num)
        # rospy.spin()

if __name__ == '__main__':
    try:
        hhhh()

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