# !/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander, multiprocessing, psutil

UR3_Q1 = [-1.294065300618307, -0.4414284865008753, -1.7840340773211878, -0.8931124846087855, 1.5558310747146606, 4.68846321105957]
UR10_Q1_to_left = [-0.1647556463824671, -1.4304211775409144, 0.9728412628173828, -1.6566603819476526, -1.4693673292743128, -2.499138418828146]

class UR10_reset:
    def ur10_reset_work(self):
        ur10_reset_process = multiprocessing.Process(target=self.initial_ur10_group, args=())
        ur10_reset_process.start()
        ur10_reset_process.join()

    def initial_ur10_group(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur10_reset", anonymous=True, disable_signals=True)

        reference_frame = "body_link"
        ur10_group_name = "ur10_with_gripper"
        self.ur10_arm_spoon = moveit_commander.MoveGroupCommander(ur10_group_name, ns='ur10')

        # 当运动规划失败后，允许重新规划
        self.ur10_arm_spoon.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        self.ur10_arm_spoon.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.ur10_arm_spoon.set_goal_position_tolerance(0.001)
        self.ur10_arm_spoon.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.ur10_arm_spoon.set_max_acceleration_scaling_factor(0.1)
        self.ur10_arm_spoon.set_max_velocity_scaling_factor(0.1)

        # 获取终端link的名称RO
        self.ur10_spoon_end_effector_link = self.ur10_arm_spoon.get_end_effector_link()

        self.ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
        self.ur10_arm_spoon.go(wait=True)




class UR3_reset:
    def ur3_reset_work(self):
        ur3_reset_process = multiprocessing.Process(target=self.initial_ur3_group, args=())
        ur3_reset_process.start()
        ur3_reset_process.join()

    def initial_ur3_group(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("ur3_reset", anonymous=True, disable_signals=True)
        reference_frame = "body_link"

        # 初始化UR3规划组
        ur3_group_name = "ur3_with_gripper"
        self.ur3_arm_ag95 = moveit_commander.MoveGroupCommander(ur3_group_name, ns="ur3")
        # 当运动规划失败后，允许重新规划
        self.ur3_arm_ag95.allow_replanning(True)
        self.ur3_arm_ag95.set_planning_time(1.0)

        # 设置目标位置所使用的参考坐标系
        self.ur3_arm_ag95.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.ur3_arm_ag95.set_goal_position_tolerance(0.001)
        self.ur3_arm_ag95.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.ur3_arm_ag95.set_max_acceleration_scaling_factor(0.1)
        self.ur3_arm_ag95.set_max_velocity_scaling_factor(0.1)

        self.ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        self.ur3_arm_ag95.go(wait=True)



if __name__ == '__main__':
    x = UR3_reset()
    x.ur3_reset_work()
