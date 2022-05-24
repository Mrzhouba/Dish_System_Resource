#encoding=utf-8
import rospy
import copy
import time
import math
import rospy
import numpy as np
import random
import actionlib
import threading
import moveit_commander
import tf
from geometry_msgs.msg import PoseStamped, Pose, PointStamped

rospy.init_node("bowl_tracking", anonymous=True, disable_signals=True)
# 初始化UR3规划组
ur3_group_name = "manipulator"
ur3_arm = moveit_commander.MoveGroupCommander(ur3_group_name)

# 当运动规划失败后，允许重新规划
ur3_arm.allow_replanning(False)

# 设置目标位置所使用的参考坐标系
reference_frame = 'base_link'
ur3_arm.set_pose_reference_frame(reference_frame)

# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
ur3_arm.set_goal_position_tolerance(0.001)
ur3_arm.set_goal_orientation_tolerance(0.001)

# 设置允许的最大速度和加速度
ur3_arm.set_max_acceleration_scaling_factor(0.05)
ur3_arm.set_max_velocity_scaling_factor(0.05)

# 获取终端link的名称
ur3_end_effector_link = ur3_arm.get_end_effector_link()

ur3_target_pos_1 = [-0.0034292379962366226, -1.601410214100973, -0.8481739203082483, -1.6790865103351038, -0.015293900166646779, 0.00037076196167618036]

# ur3_arm.set_joint_value_target(ur3_target_pos_1)
# ur3_arm.go()
current_pose = ur3_arm.get_current_pose()
print current_pose

bowl_tf_listener = tf.TransformListener()

bowl_tf_listener.waitForTransform('/base_link', '/tool0', rospy.Time(), rospy.Duration(1.0))


try:
    (trans, rot) = bowl_tf_listener.lookupTransform('/base_link', '/tool0', rospy.Time(0))
except:
    pass
print trans
print rot

bowl_pose = PoseStamped()
bowl_pose.header.frame_id = reference_frame
bowl_pose.header.stamp = rospy.Time(0)
bowl_pose.pose.position.x = trans[0]
bowl_pose.pose.position.y = trans[1]
bowl_pose.pose.position.z = trans[2] - 0.02

bowl_pose.pose.orientation.x = rot[0]
bowl_pose.pose.orientation.y = rot[1]
bowl_pose.pose.orientation.z = rot[2]
bowl_pose.pose.orientation.w = rot[3]
#
ur3_arm.set_pose_target(bowl_pose, ur3_end_effector_link)
ur3_arm.go()