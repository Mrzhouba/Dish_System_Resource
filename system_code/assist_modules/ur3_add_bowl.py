#!/usr/bin/env python
#coding=UTF-8
import os
import tf
import sys
import copy
import time
import math
import rospy
import numpy
import random
import actionlib
import threading
import moveit_commander
import moveit_msgs.msg
import cv2, cv_bridge
import roslib;
import paho.mqtt.client as mqtt
# roslib.load_manifest('ur_driver')
# roslib.load_manifest('dh_hand_driver')

from robot_order.msg import food_msg
import requests
import json

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
from geometry_msgs.msg import Twist
from moveit_msgs.msg import RobotTrajectory
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import matplotlib.pyplot as plt
import numpy as np
from geomdl import fitting
from geomdl import exchange
from geomdl.visualization import VisMPL

from io import *
from ctypes import *
from ur_msgs.msg import *
from control_msgs.msg import *
from trajectory_msgs.msg import *
from trajectory_plan import trajectory_plan
from grippers_control import grippers_control

grippers_control = grippers_control()

# ur3_catch_bowl_ready_location = [-2.6656232515918177, -0.855659310017721, -2.243136231099264, 0.0005681514739990234, 1.1339683532714844, 4.695223808288574]
# ur3_catch_bowl_location = [-2.882728878651754, -1.5910537878619593, -1.8029239813434046, 0.2891277074813843, 1.283958077430725, 4.7030415534973145]
# ur3_send_bowl_location = [-1.1535890738116663, -1.5507662932025355, -0.8751485983477991, -0.64453632036318, 1.2501745223999023, 4.701672554016113]
# ur3_lay_bowl_location = [-1.3981483618365687, -1.5980938116656702, -1.1050217787372034, -0.4047616163836878, 1.6068137884140015, 4.73676061630249]
# ur3_exit_bowl_area = [-1.4246299902545374, -1.0150006453143519, -1.4373181501971644, -0.6805298964129847, 1.6523056030273438, 4.736640453338623]
UR3_Q1 = [-1.294065300618307, -0.4414284865008753, -1.7840340773211878, -0.8931124846087855, 1.5558310747146606, 4.68846321105957]

# ur3_catch_bowl_ready_location1 = [-2.691861931477682, -1.3842123190509241, -2.2689436117755335, 0.4336811304092407, 1.1247715950012207, 4.7302045822143555]
#
#
# ur3_catch_bowl_ready_location2 = [-2.3367889563189905, -1.5242703596698206, -1.680800739918844, -0.2483885923968714, 1.525922179222107, 4.7018046379089355]

ur3_multi1_catch_bowl_ready_location = [-2.721154753361837, -1.643240753804342, -1.726382080708639, 0.2404012680053711, 1.5811971426010132, 4.726001739501953]
ur3_multi2_catch_bowl_ready_location = [-3.0613835493670862, -1.5367935339557093, -1.7715333143817347, 0.180672287940979, 1.5983432531356812, 4.733086109161377]



# ur3_catch_bowl_location = [-2.6447141806231897, -2.0172751585589808, -1.9035351912127894, 0.7955919504165649, 1.1252150535583496, 4.731693267822266]
ur3_multi1_catch_bowl_location = [-2.720675532017843, -1.8840978781329554, -2.0482099691974085, 0.8029826879501343, 1.5793399810791016, 4.72450065612793]
ur3_multi2_catch_bowl_location = [-3.0607009569751185, -1.8407767454730433, -2.175638977681295, 0.8888124227523804, 1.5959826707839966, 4.731116771697998]



# ur3_add_bowl_ready_loc = [-2.259097401295797, -1.2138474623309534, -2.1212919394122522, 0.21195411682128906, 0.7421268820762634, 4.726145267486572]
ur3_add_bowl_ready_loc_copy = [-2.1978238264666956, -0.36113006273378545, -2.433175865803854, -0.349128548298971, 1.3445783853530884, 4.71613073348999]
ur3_add_bowl_loc = [-2.644582811986105, -1.8560393492328089, -1.7746489683734339, 0.5049484968185425, 1.1262943744659424, 4.732761859893799]

ur3_send_bowl_loc_ready = [-1.9327948729144495, -1.397144619618551, -1.5861628691302698, -0.10096866289247686, 1.3588305711746216, 4.712408065795898]
ur3_device_up_loc = [-1.256916348134176, -1.4438918272601526, -1.431624714528219, -0.26722890535463506, 1.4139180183410645, 4.723455905914307]

ur3_place_loc_2 = [-1.2581022421466272, -1.4125435988055628, -1.7423113028155726, 0.012144088745117188, 1.4129348993301392, 4.723395824432373]



class ur3_add_bowl():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("ur3_add_bowl", anonymous=True, disable_signals=True)
        reference_frame = "body_link"

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
        joint_name = self.ur3_arm.get_joints()
        print joint_name


        # self.ur3_arm.set_joint_value_target(UR3_Q1)
        # self.ur3_arm.go()

        # self.ur3_arm.set_joint_value_target(ur3_multi2_catch_bowl_ready_location)
        # self.ur3_arm.go()

        self.ur3_arm.set_joint_value_target(ur3_multi1_catch_bowl_location)
        self.ur3_arm.go()

        # rospy.sleep(111111111111111)

        # ur3_add_bowl_plan = self.read_tra_message(
        #     '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
        #     '/saves_trajectory_mess/add_bowl_trajectory/xinchang_ur3_to_get_bowl_plan.json')
        ur3_multi1_add_bowl_to_device_plan_2, ur3_multi1_place_loc_2 = self.read_tra_message_with_place('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/add_bowl_trajectory'
                                         '/xinchang_multi1_ur3_to_device_up_plan_2_with_loc.json')
        print ur3_multi1_add_bowl_to_device_plan_2

        ur3_multi2_add_bowl_to_device_plan_2, ur3_multi2_place_loc_2 = self.read_tra_message_with_place('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/add_bowl_trajectory'
                                         '/xinchang_multi2_ur3_to_device_up_plan_2_with_loc.json')

        # self.ur3_arm.execute(ur3_add_bowl_plan)
        # rospy.sleep(0.2)

        self.ur3_arm.execute(ur3_multi1_add_bowl_to_device_plan_2)

        self.ur3_arm.set_joint_value_target(ur3_multi1_place_loc_2)
        self.ur3_arm.go()

        self.ur3_arm.set_joint_value_target(UR3_Q1)
        self.ur3_arm.go()

        # self.ur3_arm.set_joint_value_target(ur3_multi2_catch_bowl_ready_location)
        # self.ur3_arm.go()
        #
        # self.ur3_arm.set_joint_value_target(ur3_multi2_catch_bowl_location)
        # self.ur3_arm.go()
        #
        # self.ur3_arm.execute(ur3_multi2_add_bowl_to_device_plan_2)
        #
        # self.ur3_arm.set_joint_value_target(ur3_multi2_place_loc_2)
        # self.ur3_arm.go()
        #
        # self.ur3_arm.set_joint_value_target(UR3_Q1)
        # self.ur3_arm.go()

        # self.ur3_arm.set_joint_value_target(UR3_Q1)
        # self.ur3_arm.go()

        # self.ur3_arm.set_joint_value_target(UR3_Q1)
        # self.ur3_arm.go()
        # AG95_client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)
        # rospy.sleep(0.2)

        # self.ur3_arm.set_joint_value_target(UR3_Q1)
        # self.ur3_arm.go()

        # grippers_control.AG95_gripper(100, 100, AG95_client)
        # rospy.sleep(0.2)
        # rospy.sleep(111111111)

        # ur3_to_add_bowl_joint_list = [ur3_multi2_catch_bowl_location, ur3_send_bowl_loc_ready, ur3_device_up_loc]
        # ur3_to_add_bowl_plan = self.limit_time_toppra(ur3_to_add_bowl_joint_list, 2, joint_name, True)
        # self.save_tra_message_with_loc(ur3_to_add_bowl_plan, '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess'
        #                                             '/add_bowl_trajectory/xinchang_multi2_ur3_to_device_up_plan_2_with_loc.json')
        # self.ur3_arm.execute(ur3_to_add_bowl_plan)
        # rospy.sleep(111111)
        # # self.ur3_arm.set_joint_value_target(ur3_catch_bowl_ready_location)
        # self.ur3_arm.go()


        # ur3_to_send_bowl_joint_list = [ur3_catch_bowl_location, ur3_send_bowl_loc_ready, ur3_device_up_loc]
        # ur3_to_send_bowl_plan = self.limit_time_toppra(ur3_to_send_bowl_joint_list, 2, joint_name, True)
        # self.save_tra_message(ur3_to_add_bowl_plan, '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess'
        #                                             '/add_bowl_trajectory/xinchang_ur3_to_get_bowl_plan.json')
        # self.save_tra_message_with_loc(ur3_to_send_bowl_plan, '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess'
        #                                             '/add_bowl_trajectory/xinchang_ur3_to_device_up_plan_2_with_loc.json')


        # rospy.sleep(1111111111)

        # self.ur3_arm.execute(ur3_to_send_bowl_plan)


        # self.ur3_arm.set_joint_value_target(ur3_catch_bowl_ready_location)
        # self.ur3_arm.go()

        # ur3_add_bowl_plan = self.read_tra_message(
        #     '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
        #     '/saves_trajectory_mess/add_bowl_trajectory/xinchang_ur3_to_get_bowl_plan.json')

        # ur3_send_bowl_plan1 = self.read_tra_message(
        #     '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
        #     '/saves_trajectory_mess/add_bowl_trajectory/xinchang_multi1_ur3_to_device_up_plan_2.json')
        #
        # ur3_send_bowl_plan2 = self.read_tra_message(
        #     '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
        #     '/saves_trajectory_mess/add_bowl_trajectory/xinchang_multi2_ur3_to_device_up_plan_2.json')
        #
        # self.ur3_arm.execute(ur3_send_bowl_plan)
        #
        # grippers_control.AG95_gripper(100, 0, AG95_client)
        # rospy.sleep(0.2)
        #
        # self.ur3_arm.execute(ur3_send_bowl_plan)
        #
        # self.ur3_arm.set_joint_value_target(ur3_place_loc_2)
        # self.ur3_arm.go()
        #
        # grippers_control.AG95_gripper(100, 88, AG95_client)
        # rospy.sleep(0.2)
        #
        # self.ur3_arm.set_joint_value_target(UR3_Q1)
        # self.ur3_arm.go()

        rospy.sleep(11111111111111)

        # grippers_control.AG95_gripper(100, 100, AG95_client)

        # self.ur3_arm.set_joint_value_target(ur3_catch_bowl_location)
        # self.ur3_arm.go()
        #
        # grippers_control.AG95_gripper(100, 0, AG95_client)
        #
        # self.ur3_arm.set_joint_value_target(ur3_send_bowl_location)
        # self.ur3_arm.go()
        #
        # self.ur3_arm.set_joint_value_target(ur3_lay_bowl_location)
        # self.ur3_arm.go()

        # grippers_control.AG95_gripper(100, 100, AG95_client)
        #
        # self.ur3_arm.set_joint_value_target(ur3_exit_bowl_area)
        # self.ur3_arm.go()

    def read_tra_message(self, file_path):
        file = open(file_path)
        trajectory_mess = json.load(file)
        # print trajectory_mess
        num_points = len(trajectory_mess)
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = trajectory_mess[0]['joint_names']

        for i in range(1, num_points):
            point = JointTrajectoryPoint()
            point.positions = trajectory_mess[i]['position']
            point.velocities = trajectory_mess[i]['velocities']
            point.time_from_start.secs = trajectory_mess[i]['secs']
            point.time_from_start.nsecs = trajectory_mess[i]['nsecs']
            plan.joint_trajectory.points.append(point)

        return plan

    def limit_time_toppra(self, pos, move_time, joint_name, is_show):
        num = len(pos)
        # print pos
        tt = np.linspace(0, 1, num)
        # print len(tt)
        # print len(pos)
        path = ta.SplineInterpolator(tt, pos)

        vlim_ = np.asarray([3.5, 3.5, 3.5, 4, 4, 4])
        vlim = np.vstack((-vlim_, vlim_)).T
        alim_ = np.asarray([3.5, 3.5, 3.5, 4, 4, 6.6])
        alim = np.vstack((-alim_, alim_)).T

        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim,
                                                        discretization_scheme=constraint.DiscretizationType.Interpolation)

        instance = algo.TOPPRAsd([pc_vel, pc_acc], path)
        instance.set_desired_duration(move_time)
        jnt_traj = instance.compute_trajectory(0, 0)

        ts_sample = np.linspace(0, jnt_traj.duration, 50)
        qs_sample = jnt_traj(ts_sample)
        # print qs_sample.shape
        qds_sample = jnt_traj(ts_sample, 1)
        qdds_sample = jnt_traj(ts_sample, 2)
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = joint_name

        for i in range(50):
            points = JointTrajectoryPoint()
            for j in range(6):
                points.positions.append(qs_sample[i, j])
                points.velocities.append(qds_sample[i][j])
                points.accelerations.append(qdds_sample[i][j])
            points.time_from_start.secs = math.modf(ts_sample[i])[1]
            points.time_from_start.nsecs = math.modf(ts_sample[i])[0] * 1000000000

            plan.joint_trajectory.points.append(points)
        # print plan
        # print (new_plan.joint_trajectory.points[1])
        if is_show == True:
            fig, axs = plt.subplots(3, 1, sharex=True)
            for i in range(path.dof):
                # plot the i-th joint trajectory
                axs[0].plot(ts_sample, qs_sample[:, i])
                axs[1].plot(ts_sample, qds_sample[:, i])
                axs[2].plot(ts_sample, qdds_sample[:, i])
            axs[2].set_xlabel("Time (s)")
            axs[0].set_ylabel("Position (rad)")
            axs[1].set_ylabel("Velocity (rad/s)")
            axs[2].set_ylabel("Acceleration (rad/s2)")
            plt.xlim(0, jnt_traj.duration)
            plt.show()
            # print plan
        return plan

    def save_tra_message(self, plan, file_path):
        # self.save_tra_message(to_bowl_plan)
        joint_name = plan.joint_trajectory.joint_names
        num = len(plan.joint_trajectory.points)
        # np_data1 = np.asarray(plan.joint_trajectory.points.positions)
        result = []
        dict_name = {'joint_names': joint_name}
        result.append(dict_name)
        for i in range(num):
            dict = {'position': plan.joint_trajectory.points[i].positions,
                    'velocities': plan.joint_trajectory.points[i].velocities,
                    'accelerations': plan.joint_trajectory.points[i].accelerations,
                    'secs': plan.joint_trajectory.points[i].time_from_start.secs,
                    'nsecs': plan.joint_trajectory.points[i].time_from_start.nsecs}
            result.append(dict)
        print len(result)
        filename = file_path
        with open(filename, 'wb') as f:
            json.dump(result, f, ensure_ascii=False)

    def save_tra_message_with_loc(self, plan, file_path):
        # self.save_tra_message(to_bowl_plan)
        joint_name = plan.joint_trajectory.joint_names
        num = len(plan.joint_trajectory.points)
        # np_data1 = np.asarray(plan.joint_trajectory.points.positions)
        result = []
        dict_name = {'joint_names': joint_name}
        result.append(dict_name)
        for i in range(num):
            dict = {'position': plan.joint_trajectory.points[i].positions,
                    'velocities': plan.joint_trajectory.points[i].velocities,
                    'accelerations': plan.joint_trajectory.points[i].accelerations,
                    'secs': plan.joint_trajectory.points[i].time_from_start.secs,
                    'nsecs': plan.joint_trajectory.points[i].time_from_start.nsecs}
            result.append(dict)
        place_loc = {'ur3_place_loc': ur3_place_loc_2}
        result.append(place_loc)
        print len(result)
        filename = file_path
        with open(filename, 'wb') as f:
            json.dump(result, f, ensure_ascii=False)


    def read_tra_message_with_place(self, file_path):
        file = open(file_path)
        trajectory_mess = json.load(file)
        # print trajectory_mess
        num_points = len(trajectory_mess)
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = trajectory_mess[0]['joint_names']
        ur3_bowl_place_loc = trajectory_mess[-1]['ur3_place_loc']

        for i in range(1, num_points-1):
            point = JointTrajectoryPoint()
            point.positions = trajectory_mess[i]['position']
            point.velocities = trajectory_mess[i]['velocities']
            point.accelerations = trajectory_mess[i]['accelerations']
            point.time_from_start.secs = trajectory_mess[i]['secs']
            point.time_from_start.nsecs = trajectory_mess[i]['nsecs']
            plan.joint_trajectory.points.append(point)

        return plan, ur3_bowl_place_loc


if __name__ == '__main__':

    try:
        ur3_add_bowl()

    except rospy.ROSInterruptException:
        pass