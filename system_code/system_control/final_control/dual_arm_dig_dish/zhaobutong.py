#!/usr/bin/env python
#coding=UTF-8
import rospy
import moveit_commander
import sys, math, copy
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pointnet2_pytorch/utils')
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/assist_modules')

# from get_model_output import get_model_output
import Variable
import pyrealsense2 as rs
from sensor_msgs.msg import JointState,Image,CameraInfo, PointCloud2
import open3d as o3d
import numpy as np
import numpy, json
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from open3d_ros_helper import open3d_ros_helper as orh
from geomdl import fitting
from trajectory_msgs.msg import *
import tf, time
import paho.mqtt.client as mqtt
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal
import actionlib
from moveit_msgs.msg import PlanningScene
from moveit_python import *
import multiprocessing, os
import psutil, serial, binascii
from cv_bridge import CvBridge,CvBridgeError
import keyboard
import pykdl_utils
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from geometry_msgs.msg import PoseStamped, Pose, Point
from final_judge_dish_location_qt import final_judge_dish_location_qt

final_judge_dish_location_qt = final_judge_dish_location_qt()

base_path = Variable.base_path
# weight_path = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pointnet2_pytorch/log/dish/checkpoints/pointnet2_seg_1000_acc_0.9461.pth'
# weight_path = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pointnet2_pytorch/log/dish_diff_depth/xyz_5000_pre_down/checkpoints/pointnet2_seg_802_acc_0.4189.pth'
# weight_path = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pointnet2_pytorch/log/dish_diff_depth/xyz_2500_pre_down/checkpoints/pointnet2_seg_449_acc_0.4063.pth'
# weight_path = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pointnet2_pytorch/log/dish_diff_depth/all/pointnet2_seg_445_acc_0.4087.pth'
# weight_path = base_path + 'pointnet++_log/2500_3_label/checkpoints/pointnet2_seg_300_acc_0.5473.pth'
# weight_path = base_path + 'pointnet++_log/dish_log/all_2500_200/checkpoints/pointnet2_seg_290_acc_0.8751_iou_0.5519.pth'

# weight_path = base_path + 'dgcnn_log/dgcnn_all_dish_300/models/epoch_280_acc_0.872781_iou_0.753227.pth'
# weight_path = base_path + 'dgcnn_log/dgcnn_resize/models/epoch_288_acc_0.873824_iou_0.746191.pth'
weight_path = Variable.weight_path
#
# weight_path = base_path + 'dgg_log/dgg_resize/models/epoch_275_acc_0.869592_iou_0.739159.pth'
# weight_path = base_path + 'dgg_log/dgg_all_dish_300/models/epoch_280_acc_0.865356_iou_0.739130.pth'
# weight_path = base_path + 'dgg_log/dgg_only_dig_300/models/epoch_265_acc_0.868822_iou_0.768025.pth'
# weight_path = base_path + 'dgg_log/dy_double_covar_all_dish_300/models/epoch_285_acc_0.871286_iou_0.746431.pth'
# weight_path = base_path + 'dgg_log/dy_double_covar_all_dish_300/models/epoch_285_acc_0.871286_iou_0.746431.pth'
model_name = Variable.model_name
channels = Variable.channels
nclasses = Variable.nclasses

UR3_Q1 = Variable.UR3_Q1
UR3_lay_location = Variable.UR3_lay_location

one_bowl_loc = Variable.one_bowl_loc
two_bowl_loc = Variable.two_bowl_loc

one_bowl_up_loc = Variable.one_bowl_up_loc
two_bowl_up_loc = Variable.two_bowl_up_loc

UR10_pull = Variable.UR10_pull
UR10_Q1_to_left = Variable.UR10_Q1_to_left
UR10_Q1 = Variable.UR10_Q1

ur10_max_acceleration = Variable.ur10_max_acceleration
ur10_max_velocity = Variable.ur10_max_velocity

camera_angele = Variable.camera_angele
empty_depth = Variable.empty_depth

volume_goal = Variable.require_volume
max_depth = Variable.empty_depth - Variable.real_max_depth
ur10_pull_dish_tra_path = Variable.ur10_pull_dish_tra_path

cut_safe_area_xbound_min = Variable.cut_safe_area_xbound_min
cut_safe_area_xbound_max = Variable.cut_safe_area_xbound_max
cut_safe_area_ybound_min = Variable.cut_safe_area_ybound_min
cut_safe_area_ybound_max = Variable.cut_safe_area_ybound_max
cut_safe_area_z = Variable.cut_safe_area_z

pause_usb_name = Variable.pause_usb_name

# empty_depth = 0.6035540679
# empty_depth = 0.638144
# 0.638144

class model_effect_test():
    def __init__(self):
        self.camera_angele = camera_angele

        fn = sys.stdin.fileno()

        # signal_ur3_send, signal_ur3_receive = multiprocessing.Pipe(True)
        # signal_ur10_send, signal_ur10_receive = multiprocessing.Pipe(True)

        dig_dish = multiprocessing.Process(target=self.dig_dish, args=(fn,))
        # get_bowl = multiprocessing.Process(target=self.ur3_get_bowl, args=(signal_ur3_send, signal_ur10_receive))
        dig_dish.start()
        # get_bowl.start()
        dig_dish_pid = dig_dish.pid
        # get_bowl_pid = get_bowl.pid

        pause_process = multiprocessing.Process(target=self.pause_process, args=(dig_dish_pid,))
        # pause_process.start()

        dig_dish.join()
        # pause_process.join()

    def pause_process(self, dig_dish_pid):
        dish_pause = psutil.Process(dig_dish_pid)
        # bowl_pause = psutil.Process(get_bowl_pid)
        portx = pause_usb_name
        bps = 9600
        timex = None
        ser = serial.Serial(portx, bps, timeout=timex)
        while 1:
            data = binascii.b2a_hex(ser.read())
            if data == "13":
                dish_pause.suspend()
                # bowl_pause.suspend()
                print('打菜暂停运行')

            data = binascii.b2a_hex(ser.read())
            if data == "13":
                dish_pause.resume()
                # bowl_pause.resume()
                print('打菜恢复运行')

    def dig_dish(self, fn):
        from get_model_output import get_model_output
        get_model_output = get_model_output(model_name, channels, nclasses, weight_path)
        sys.stdin = os.fdopen(fn)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("model_effect", anonymous=True, disable_signals=True)

        # self.ur10_scene = PlanningSceneInterface("odom")
        # self.ur3_scene = PlanningSceneInterface("odom", ns="ur3")

        reference_frame = "body_link"
        ur10_group_name = "ur10_with_gripper"
        self.ur10_arm_spoon = moveit_commander.MoveGroupCommander(ur10_group_name)


        # 当运动规划失败后，允许重新规划
        self.ur10_arm_spoon.allow_replanning(False)
        self.ur10_arm_spoon.set_planning_time(1.0)

        # 设置目标位置所使用的参考坐标系
        self.ur10_arm_spoon.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.ur10_arm_spoon.set_goal_position_tolerance(0.001)
        self.ur10_arm_spoon.set_goal_orientation_tolerance(0.001)
        # self.ur10_arm_spoon.set_goal_joint_tolerance(0.01)
        # print self.ur10_arm_spoon.get_goal_joint_tolerance()
        # print self.ur10_arm_spoon.get_goal_tolerance()

        # 设置允许的最大速度和加速度
        self.ur10_arm_spoon.set_max_acceleration_scaling_factor(ur10_max_acceleration)
        self.ur10_arm_spoon.set_max_velocity_scaling_factor(ur10_max_velocity)

        # 获取终端link的名称
        self.ur10_spoon_end_effector_link = self.ur10_arm_spoon.get_end_effector_link()
        print '终端link的名称', self.ur10_spoon_end_effector_link
        # rospy.sleep(1000000)
        # self.add_desk_belt()
        # rospy.set_param('/position_based_position_trajectory_controller/stop_trajectory_duration',
        #                 0.5)
        self.ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
        self.ur10_arm_spoon.go()

        self.count = -1
        self.tempCount = -1

        self.test = False

        if self.test == True:
            self.is_save_cloud = False
            self.is_dig = False
        else:
            self.is_save_cloud = False
            self.is_dig = True

        self.is_normalize = False

        self.ur10_pull_dish_plan = self.read_tra_message(ur10_pull_dish_tra_path)

        args = [self.ur10_arm_spoon, self.ur10_spoon_end_effector_link, 0, 0, get_model_output]

        # args = [signal_ur10_send, signal_ur3_receive, get_model_output]
        for _ in range(5):
            start_signal = 'dig start'
            completed = False

            if start_signal == 'dig start':
                self.count += 1
                while not completed:
                    point_cloud_data = rospy.wait_for_message("/camera2/depth/color/points", PointCloud2, timeout=1)
                    completed = final_judge_dish_location_qt.points_callback(point_cloud_data, args)
                    print completed


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


if __name__ == '__main__':
    x = model_effect_test()