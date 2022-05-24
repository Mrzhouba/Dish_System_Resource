#!/usr/bin/env python
#coding=UTF-8
import tf
import rospy,sys
import moveit_commander
import roslib; roslib.load_manifest('ur_driver')
import open3d as o3d
import paho.mqtt.client as mqtt
import copy
import json
import actionlib
import math
import numpy as np
import constant
import matplotlib.pyplot as plt
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from geometry_msgs.msg import Pose, PoseStamped
import multiprocessing
import threading
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import sys
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/assist_modules')
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal
from grippers_control import grippers_control
grippers_control = grippers_control()

UR3_Q1 = [-1.294065300618307, -0.4414284865008753, -1.7840340773211878, -0.8931124846087855, 1.5558310747146606, 4.68846321105957]
UR3_Q2 = [-1.2820218245135706, -0.6487200895892542, -2.0104287306415003, -1.0251391569720667, 1.5584312677383423, 4.688642978668213]
UR3_Q3 = [-1.2514274756061, -1.585510555897848, -2.2166693846331995, 0.6478713750839233, 1.4034786224365234, 4.688450813293457]
UR3_Q3_1 = [-1.250984017048971, -1.5978778044330042, -2.2185753027545374, 0.672469973564148, 1.401285171508789, 4.688378810882568]
UR3_Q4 = [0.15033797919750214, -1.7251623312579554, -2.0601723829852503, 0.6312607526779175, 1.3875133991241455, 4.6887993812561035]

UR3_bowl_start = [-0.711865250264303, -1.5908144156085413, -2.2174962202655237, 0.6579166650772095, 1.4031790494918823, 4.688511371612549]
UR3_bowl_start_1 = [-0.7330239454852503, -1.4046404997455042, -1.6935294310199183, -0.0522082487689417, 1.406115174293518, 4.689796447753906]
class ur3_get_bowl:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur3_get_bowl", anonymous=True)

        reference_frame = "base_link"
        # 初始化UR10规划组
        ur3_group_name = "ur3_with_gripper"
        ur3_arm_ag95 = moveit_commander.MoveGroupCommander(ur3_group_name, ns="ur3")

        # 当运动规划失败后，允许重新规划
        ur3_arm_ag95.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        ur3_arm_ag95.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        ur3_arm_ag95.set_goal_position_tolerance(0.001)
        ur3_arm_ag95.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        ur3_arm_ag95.set_max_acceleration_scaling_factor(0.1)
        ur3_arm_ag95.set_max_velocity_scaling_factor(0.1)

        ur3_ag95_end_effector_link = ur3_arm_ag95.get_end_effector_link()
        # ur3_arm_ag95.set_planner_id("RRTConnect")

        joint_name = ur3_arm_ag95.get_joints()
        # print type(joint_name)

        # ur3_arm_ag95.set_joint_value_target(UR3_Q4)
        # ur3_arm_ag95.go()
        # ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        # ur3_arm_ag95.go()

        # to_bowl_joint_list = [UR3_Q1, UR3_Q2, UR3_Q3]
        # to_bowl_plan = self.toppra(to_bowl_joint_list, 5, joint_name, False)
        # print(type(to_bowl_plan))
        # print to_bowl_plan.joint_trajectory.joint_names
        #
        # ur3_arm_ag95.execute(to_bowl_plan)

        to_bowl_plan = self.read_tra_message('./get_bowl_trajectory/ur3_to_bowl_trajectory.json')
        # ur3_arm_ag95.execute(to_bowl_plan)

        # to_start_joint_list = [UR3_Q3_1, UR3_bowl_start, UR3_bowl_start_1, UR3_Q1]
        # to_start_plan = self.toppra(to_start_joint_list, 5, joint_name, False)
        # ur3_arm_ag95.execute(to_start_plan)
        #
        # self.save_tra_message(to_bowl_plan, './ur3_to_bowl_trajectory.json')
        # self.save_tra_message(to_start_plan, '../saves_trajectory_mess/ur3_to_start_trajectory.json')
        plan = self.read_tra_message('./get_bowl_trajectory/ur3_to_start_trajectory.json')
        for i in range(100):
            ur3_arm_ag95.execute(plan)

            ur3_arm_ag95.execute(to_bowl_plan)



        # ur3_arm_ag95.set_joint_value_target(UR3_Q4)
        # ur3_arm_ag95.go()

        # self.plot_curve(plan)

    def toppra(self, pos, move_time, joint_name, is_show):
        num = len(pos)
        # print pos
        tt = np.linspace(0, 1, num)
        # print len(tt)
        # print len(pos)
        path = ta.SplineInterpolator(tt, pos)
        # [3, 3, 3, 3, 3, 3]
        vlim_ = np.asarray([5, 1, 5, 5, 5, 5])
        vlim = np.vstack((-vlim_, vlim_)).T
        alim_ = np.asarray([5, 1, 5, 5, 5, 5])
        alim = np.vstack((-alim_, alim_)).T

        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim,
                                                        discretization_scheme=constraint.DiscretizationType.Interpolation)

        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        # instance.set_desired_duration(1.5)
        jnt_traj = instance.compute_trajectory(0, 0)

        ts_sample = np.linspace(0, jnt_traj.duration, 100)
        qs_sample = jnt_traj(ts_sample)
        # print qs_sample.shape
        qds_sample = jnt_traj(ts_sample, 1)
        qdds_sample = jnt_traj(ts_sample, 2)
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = joint_name
        # print plan

        for i in range(100):
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
        if is_show == True:
            plt.show()
        # print plan
        return plan

    def plot_curve(self, plan):

        # plan = self.five_limit_time_plan(plan, 5)
        num = len(plan.joint_trajectory.points)
        print num
        time = []
        pos = np.ones((6, num))
        velocity = np.ones((6, num))
        acc = np.ones((6, num))

        for i in range(num):
            time.append(plan.joint_trajectory.points[i].time_from_start.secs + plan.joint_trajectory.points[
                i].time_from_start.nsecs * 1.0 / 1000000000)
            for j in range(6):
                pos[j][i] = plan.joint_trajectory.points[i].positions[j]
                velocity[j][i] = plan.joint_trajectory.points[i].velocities[j]
                acc[j][i] = plan.joint_trajectory.points[i].accelerations[j]

        # for i in range(6):
        #     print i
        #     print max(velocity[i])
        #     print min(velocity[i])

        # np.savetxt('/home/robot/tra.txt', pos, fmt='%.7e')

        fig, axs = plt.subplots(3, 1, sharex=True)
        for i in range(6):
            # plot the i-th joint trajectory
            axs[0].plot(time, pos[i, :])
            axs[1].plot(time, velocity[i, :])
            axs[2].plot(time, acc[i, :])
        axs[2].set_xlabel("Time (s)")
        axs[0].set_ylabel("Position (rad)")
        axs[1].set_ylabel("Velocity (rad/s)")
        axs[2].set_ylabel("Acceleration (rad/s2)")
        plt.show()

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
        with open(filename, 'w') as f:
            json.dump(result, f, ensure_ascii=False)

        # print np_data
        # np.save('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/ur3_start_to_bowl.txt', np_data)
        # time = []

        # pos = np.ones((6, num))
        # velocity = np.ones((6, num))
        # acc = np.ones((6, num))
        #
        # for i in range(num):
        #     time.append(plan.joint_trajectory.points[i].time_from_start.secs + plan.joint_trajectory.points[
        #         i].time_from_start.nsecs * 1.0 / 1000000000)
        #     for j in range(6):
        #         pos[j][i] = plan.joint_trajectory.points[i].positions[j]
        #         velocity[j][i] = plan.joint_trajectory.points[i].velocities[j]
        #         acc[j][i] = plan.joint_trajectory.points[i].accelerations[j]

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

# ur3_add_bowl_ready_loc = [-2.259097401295797, -1.2138474623309534, -2.1212919394122522, 0.21195411682128906, 0.7421268820762634, 4.726145267486572]
# ur3_add_bowl_ready_loc_copy = [-2.1978238264666956, -0.36113006273378545, -2.433175865803854, -0.349128548298971, 1.3445783853530884, 4.71613073348999]
# ur3_add_bowl_loc = [-2.644582811986105, -1.8560393492328089, -1.7746489683734339, 0.5049484968185425, 1.1262943744659424, 4.732761859893799]
#
# ur3_send_bowl_loc_ready = [-1.9327948729144495, -1.397144619618551, -1.5861628691302698, -0.10096866289247686, 1.3588305711746216, 4.712408065795898]
# ur3_send_bowl_loc_up_3 = [-1.2622488180743616, -1.43774921098818, -1.4549840132342737, -0.24920255342592412, 1.413870096206665, 4.723263740539551]
# ur3_send_bowl_loc_up_4 = [-1.259312931691305, -1.4256675879107874, -1.419112507496969, -0.29847556749452764, 1.4102743864059448, 4.722987174987793]
#
# ur3_place_loc_4 = [-1.259312931691305, -1.390738312398092, -1.655079189931051, -0.09758359590639287, 1.4098550081253052, 4.72305965423584]
# ur3_place_loc_3 = [-1.2624285856830042, -1.4102090040790003, -1.722426716481344, -0.009214703236715138, 1.4132105112075806, 4.723275661468506]
# ur3_place_loc_2 = [-1.2581022421466272, -1.4125435988055628, -1.7423113028155726, 0.012144088745117188, 1.4129348993301392, 4.723395824432373]

class ur3_add_bowl:
    def __init__(self):
        move_3 = multiprocessing.Process(target=self.move_ur3, args=())
        move_10 = multiprocessing.Process(target=self.move_ur10,
                                          args=())
        move_3.start()
        move_10.start()

        move_3.join()
        move_10.join()

    def move_ur10(self):
        print 123

    def move_ur3(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur3_get_bowl", anonymous=True)

        reference_frame = "base_link"
        # 初始化UR10规划组
        ur3_group_name = "ur3_with_gripper"
        ur3_arm_ag95 = moveit_commander.MoveGroupCommander(ur3_group_name, ns="ur3")

        # 当运动规划失败后，允许重新规划
        ur3_arm_ag95.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        ur3_arm_ag95.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        ur3_arm_ag95.set_goal_position_tolerance(0.001)
        ur3_arm_ag95.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        ur3_arm_ag95.set_max_acceleration_scaling_factor(0.1)
        ur3_arm_ag95.set_max_velocity_scaling_factor(0.1)

        ur3_ag95_end_effector_link = ur3_arm_ag95.get_end_effector_link()
        # ur3_arm_ag95.set_planner_id("RRTConnect")
        AG95_client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)

        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()

        rospy.sleep(0.3)
        # grippers_control.AG95_gripper(100, 100, AG95_client)
        joint_name = ur3_arm_ag95.get_joints()

        gripper = threading.Thread(target=self.open_gripper, args=(AG95_client,))
        ur_move = threading.Thread(target=self.ur3_to_bowl, args=(ur3_arm_ag95,))
        gripper.start()
        ur_move.start()

        ur_move.join()
        gripper.join()

        rospy.sleep(0.5)
        grippers_control.AG95_gripper(100, 15, AG95_client)
        ur3_to_device_up_plan, ur3_place_loc = self.read_tra_message_with_place(
            './add_bowl_trajectory/ur3_to_device_up_plan_3.json')
        print ur3_place_loc
        ur3_arm_ag95.execute(ur3_to_device_up_plan)

        ur3_arm_ag95.set_joint_value_target(ur3_place_loc)
        ur3_arm_ag95.go()

        rospy.sleep(0.5)
        grippers_control.AG95_gripper(100, 88, AG95_client)
        #
        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()

    def open_gripper(self, AG95_client):
        # rospy.sleep(0.5)
        grippers_control.AG95_gripper(100, 100, AG95_client)


    def ur3_to_bowl(self, ur3_arm_ag95):
        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()



        # ur3_to_get_bowl_mess = [UR3_Q1, ur3_add_bowl_ready_loc_copy, ur3_add_bowl_loc]
        # ur3_to_get_bowl_plan = self.toppra(ur3_to_get_bowl_mess, 5, joint_name, True)
        # ur3_arm_ag95.execute(ur3_to_get_bowl_plan)
        # self.save_tra_message(ur3_to_get_bowl_plan, './add_bowl_trajectory/ur3_to_get_bowl_plan.json')


        ur3_to_get_bowl_plan = self.read_tra_message('./add_bowl_trajectory/ur3_to_get_bowl_plan.json')
        ur3_arm_ag95.execute(ur3_to_get_bowl_plan)



        # ur3_to_device_up_mess = [ur3_add_bowl_loc, ur3_send_bowl_loc_ready, ur3_send_bowl_loc_up_4]
        # ur3_to_device_up_plan = self.toppra(ur3_to_device_up_mess, 5, joint_name, False)
        # ur3_arm_ag95.execute(ur3_to_device_up_plan)
        #
        # self.save_tra_message(ur3_to_device_up_plan, './add_bowl_trajectory/ur3_to_device_up_plan_4.json')





    def toppra(self, pos, move_time, joint_name, is_show):
        num = len(pos)
        # print pos
        tt = np.linspace(0, 1, num)
        # print len(tt)
        # print len(pos)
        path = ta.SplineInterpolator(tt, pos)
        # [3, 3, 3, 3, 3, 3]
        vlim_ = np.asarray([2, 2, 2, 2.5, 2, 2])
        vlim = np.vstack((-vlim_, vlim_)).T
        alim_ = np.asarray([1.5, 1.5, 1.8, 2, 1.8, 1.8])
        alim = np.vstack((-alim_, alim_)).T

        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(alim,
                                                        discretization_scheme=constraint.DiscretizationType.Interpolation)

        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        # instance.set_desired_duration(1.5)
        jnt_traj = instance.compute_trajectory(0, 0)

        ts_sample = np.linspace(0, jnt_traj.duration, 100)
        qs_sample = jnt_traj(ts_sample)
        # print qs_sample.shape
        qds_sample = jnt_traj(ts_sample, 1)
        qdds_sample = jnt_traj(ts_sample, 2)
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = joint_name
        # print plan

        for i in range(100):
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
        if is_show == True:
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
        with open(filename, 'w') as f:
            json.dump(result, f, ensure_ascii=False)

        # print np_data
        # np.save('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/saves_trajectory_mess/ur3_start_to_bowl.txt', np_data)
        # time = []

        # pos = np.ones((6, num))
        # velocity = np.ones((6, num))
        # acc = np.ones((6, num))
        #
        # for i in range(num):
        #     time.append(plan.joint_trajectory.points[i].time_from_start.secs + plan.joint_trajectory.points[
        #         i].time_from_start.nsecs * 1.0 / 1000000000)
        #     for j in range(6):
        #         pos[j][i] = plan.joint_trajectory.points[i].positions[j]
        #         velocity[j][i] = plan.joint_trajectory.points[i].velocities[j]
        #         acc[j][i] = plan.joint_trajectory.points[i].accelerations[j]

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
            point.time_from_start.secs = trajectory_mess[i]['secs']
            point.time_from_start.nsecs = trajectory_mess[i]['nsecs']
            plan.joint_trajectory.points.append(point)

        return plan, ur3_bowl_place_loc


if __name__ == "__main__":
    ur3_get_bowl()
    # ur3_add_bowl()