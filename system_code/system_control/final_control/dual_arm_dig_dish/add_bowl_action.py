# !/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander, multiprocessing, psutil
import actionlib, json
import paho.mqtt.client as mqtt
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/assist_modules')
from scan_dish import scan_dish
scan_dish = scan_dish()
from grippers_control import grippers_control
grippers_control = grippers_control()

UR3_Q1 = [-1.294065300618307, -0.4414284865008753, -1.7840340773211878, -0.8931124846087855, 1.5558310747146606, 4.68846321105957]
HOST = "192.168.1.176"
PORT = 1883
add_bowl_client = mqtt.Client("add_bowl")


class ur3_add_bowl_action:
    def ur3_add_bowl_work(self, current_car_loc):
        signal_bowl_send, signal_car_receive = multiprocessing.Pipe(True)
        add_bowl_process = multiprocessing.Process(target=self.add_bowl, args=(signal_bowl_send,))
        car_process = multiprocessing.Process(target=self.car_move, args=(signal_car_receive, current_car_loc))
        add_bowl_process.start()
        car_process.start()

        add_bowl_process.join()
        car_process.join()
        print 'add bowl end process'

    def add_bowl(self, signal_bowl_send):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("add_bowl", anonymous=True, disable_signals=True)
        reference_frame = "body_link"

        # 初始化UR3规划组
        ur3_group_name = "ur3_with_gripper"
        ur3_arm_ag95 = moveit_commander.MoveGroupCommander(ur3_group_name, ns="ur3")
        # 当运动规划失败后，允许重新规划
        ur3_arm_ag95.allow_replanning(True)
        ur3_arm_ag95.set_planning_time(1.0)

        # 设置目标位置所使用的参考坐标系
        ur3_arm_ag95.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        ur3_arm_ag95.set_goal_position_tolerance(0.001)
        ur3_arm_ag95.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        ur3_arm_ag95.set_max_acceleration_scaling_factor(0.5)
        ur3_arm_ag95.set_max_velocity_scaling_factor(0.5)



        AG95_client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)
        rospy.sleep(0.4)

        # ur3_arm_ag95.set_joint_value_target(one_bowl_loc[0])
        # ur3_arm_ag95.go()
        ur3_add_bowl_plan = self.read_tra_message(
            '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
            '/saves_trajectory_mess/add_bowl_trajectory/ur3_to_get_bowl_plan.json')
        ur3_add_bowl_to_device_plan_2, ur3_place_loc_2 = self.read_tra_message_with_place(
            '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
            '/saves_trajectory_mess/add_bowl_trajectory/ur3_to_device_up_plan_2.json')
        ur3_add_bowl_to_device_plan_3, ur3_place_loc_3 = self.read_tra_message_with_place(
            '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
            '/saves_trajectory_mess/add_bowl_trajectory/ur3_to_device_up_plan_3.json')
        ur3_add_bowl_to_device_plan_4, ur3_place_loc_4 = self.read_tra_message_with_place(
            '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
            '/saves_trajectory_mess/add_bowl_trajectory/ur3_to_device_up_plan_4.json')
        ur3_add_bowl_to_device_plan = [ur3_add_bowl_to_device_plan_2, ur3_add_bowl_to_device_plan_3,
                                       ur3_add_bowl_to_device_plan_4]
        ur3_place_bowl_to_device_loc = [ur3_place_loc_2, ur3_place_loc_3, ur3_place_loc_4]

        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()

        add_bowl_signal = signal_bowl_send.recv()
        if add_bowl_signal == 'arrive target':

            grippers_control.AG95_gripper(100, 100, AG95_client)

            ur3_arm_ag95.execute(ur3_add_bowl_plan)
            rospy.sleep(0.2)
            grippers_control.AG95_gripper(100, 15, AG95_client)

            ur3_arm_ag95.execute(ur3_add_bowl_to_device_plan_2)

            ur3_arm_ag95.set_joint_value_target(ur3_place_loc_2)
            ur3_arm_ag95.go()

            rospy.sleep(0.5)
            grippers_control.AG95_gripper(100, 88, AG95_client)
            #
            signal_bowl_send.send('go back')
            ur3_arm_ag95.set_joint_value_target(UR3_Q1)
            ur3_arm_ag95.go()


    def car_move(self, signal_car_receive, current_car_loc):
        add_bowl_client.connect(HOST, PORT)
        add_bowl_client.loop_start()
        move_dis = 0.7 - current_car_loc

        scan_dish.car_move(move_dis, add_bowl_client)

        signal_car_receive.send('arrive target')

        go_back_signal = signal_car_receive.recv()
        if go_back_signal == 'go back':
            scan_dish.car_move(-move_dis, add_bowl_client)

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




if __name__ == '__main__':
    x = ur3_add_bowl_action()
    x.ur3_add_bowl_work(0)
    print 1

