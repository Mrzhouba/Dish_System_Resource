#!/usr/bin/env python
#coding=UTF-8
import math
import sys
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/assist_modules')

import rospy
import numpy
import copy
# import roslib; roslib.load_manifest('ur_driver')
import moveit_msgs.msg
import moveit_commander
import roslib
import math
import time
import tf
# roslib.load_manifest('ur_driver')
# roslib.load_manifest('dh_hand_driver')
import numpy as np
import json
import matplotlib.pyplot as plt
import Variable

from copy import deepcopy
from control_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, WrenchStamped
from geomdl import fitting
from moveit_msgs.msg import RobotTrajectory
from geomdl.visualization import VisMPL
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics



# ur10_pull = [-0.38926393190492803, -1.5942300001727503, 1.6245465278625488, -1.9102380911456507, -2.4574254194842737, 0.6636695265769958-math.pi/2]
UR10_Q1_to_left = Variable.UR10_Q1_to_left
# UR10_Q1 = [-0.16467219987978154, -1.430373493825094, 0.9728174209594727, -1.656503979359762, -1.4693554083453577, -0.9290836493121546]

# old dual-arm move together
# ur10_pull_ready = [-0.399757210408346, -1.6376188437091272, 1.705512523651123, -1.9663198629962366, -2.4519768396960657, -2.77193791071047]
# ur10_pull_ready_loc = [-0.152, -0.594, 0.844]
# ur10_pull_ready_rpy = [0.001, 0.000, -1.710]
# ur10_pull_ready_angle_y = -97.994

# new ur10-rotate-only
# ur10_pull_ready = [-0.4108622709857386, -1.7223156134234827, 1.769186019897461, -2.1063221136676233, -2.391418759022848, -2.9845884482013147]
# ur10_pull_ready_loc = [-0.140, -0.600, 0.868]
# ur10_pull_ready_rpy = [0.000, 0.000, -1.569]
# ur10_pull_ready_angle_y = -89.904

#fts
ur10_pull_ready = Variable.ur10_pull_ready
ur10_pull_ready_loc = Variable.ur10_pull_ready_loc
ur10_pull_ready_rpy = Variable.ur10_pull_ready_rpy
ur10_pull_ready_angle_y = Variable.ur10_pull_ready_angle_y

left_up_push_start_ready_fts = Variable.left_up_push_start_ready_fts
right_up_push_start_ready_fts = Variable.right_up_push_start_ready_fts

camera_angele = Variable.camera_angele

push_file_path = Variable.push_file_path

class final_trajectory_plan_qt():

    def push_action_trajectory_plan(self, action_name, waypoints, orientation_waypoints, Group, Group_end_effector):

        ur10_joint_name = Group.get_joints()
        #???????????????????????????????????????body_link???
        if action_name != 'initial push':
            transform_cameraTo_body_link = tf.TransformListener()
            transform_cameraTo_body_link.waitForTransform("/body_link", "/camera2_depth_optical_frame", rospy.Time(),
                                                          rospy.Duration(1.0))
            waypoints_temp = PoseStamped()
            waypoints_temp.header.frame_id = "camera2_depth_optical_frame"
            waypoints_temp.header.stamp = rospy.Time(0)
            waypoints_temp.pose.position.x = waypoints[0][0]
            waypoints_temp.pose.position.y = waypoints[0][1]
            waypoints_temp.pose.position.z = waypoints[0][2]
            waypoints_temp.pose.orientation.x = 0
            waypoints_temp.pose.orientation.y = 0
            waypoints_temp.pose.orientation.z = 0
            waypoints_temp.pose.orientation.w = 1

            body_link_waypoints = []
            for i in range(3):
                waypoints_temp.pose.position.x = waypoints[i][0]
                waypoints_temp.pose.position.y = waypoints[i][1]
                waypoints_temp.pose.position.z = waypoints[i][2]
                Trans_body_link = transform_cameraTo_body_link.transformPose("/body_link", waypoints_temp)
                temp_loc = [Trans_body_link.pose.position.x, Trans_body_link.pose.position.y,
                            Trans_body_link.pose.position.z - 0.01]
                body_link_waypoints.append(temp_loc)

        if action_name == 'dig dish':
            curve = fitting.interpolate_curve(waypoints, 2)
            interpolation_points = curve.evalpts

            #body_link?????????????????????
            body_link_curve = fitting.interpolate_curve(body_link_waypoints, 2)
            body_link_curve.delta = 0.1
            body_link_interpolation_points = body_link_curve.evalpts

            four_elements_list = []
            body_link_four_elements_list = []
            start_real_x, start_real_y = self.get_rotate_angle_to_end(orientation_waypoints[0], orientation_waypoints[1])
            # print orientation_waypoints[0], orientation_waypoints[1], orientation_waypoints[2]

            body_link_start_quaternion = self.compute_rotate_matrix([math.pi/2, camera_angele-math.pi, start_real_x, start_real_y, 0], 'zyxyz')
            start_rotate_xyz = self.compute_rotated_angle_y([0,1,0], [math.pi/2, camera_angele-math.pi, start_real_x, start_real_y, 0], 'zyxyz')
            # print rotate_xyz*180/math.pi
            if start_rotate_xyz < math.pi / 4:
                body_link_start_quaternion = self.compute_rotate_matrix([math.pi / 2, camera_angele - math.pi, start_real_x, start_real_y, -math.pi/4+start_rotate_xyz], 'zyxyz')

            start_quaternion = self.compute_rotate_matrix([start_real_x, start_real_y, 0], 'xyz')

            four_elements_list.append(start_quaternion)
            body_link_four_elements_list.append(body_link_start_quaternion)


            middle_real_x, middle_real_y = self.get_rotate_angle_to_end(orientation_waypoints[1], orientation_waypoints[2])

            body_link_middle_quaternion = self.compute_rotate_matrix([math.pi / 2, camera_angele - math.pi, middle_real_x, middle_real_y], 'zyxy')
            middle_rotate_xyz = self.compute_rotated_angle_y([0, 1, 0], [math.pi / 2, camera_angele - math.pi, middle_real_x, middle_real_y, 0], 'zyxyz')

            if middle_rotate_xyz < math.pi / 4:
                body_link_middle_quaternion = self.compute_rotate_matrix([math.pi / 2, camera_angele - math.pi, middle_real_x, middle_real_y, -math.pi/4+middle_rotate_xyz], 'zyxyz')
            # print middle_rotate_xyz*180/math.pi

            middle_quaternion = self.compute_rotate_matrix([middle_real_x, middle_real_y], 'xy')

            body_link_four_elements_list.append(body_link_middle_quaternion)
            four_elements_list.append(middle_quaternion)

            theta_z = math.atan2(waypoints[2][0] - waypoints[0][0], waypoints[2][1] - waypoints[0][1])
            end_real_x, end_real_y = self.get_rotate_angle_to_end(orientation_waypoints[2], orientation_waypoints[3])

            body_link_end_quaternion = self.compute_rotate_matrix([math.pi / 2, camera_angele - math.pi, end_real_x, end_real_y, math.pi/2+theta_z, 20*math.pi/180],
                                                                  'zyxyzy')
            end_quaternion = self.compute_rotate_matrix([end_real_x, end_real_y, math.pi/2+theta_z], 'xyz')

            body_link_four_elements_list.append(body_link_end_quaternion)
            four_elements_list.append(end_quaternion)


            body_start_point = PoseStamped()
            body_start_point.header.frame_id = "body_link"
            body_start_point.header.stamp = rospy.Time(0)
            body_start_point.pose.position.x = body_link_waypoints[0][0]
            body_start_point.pose.position.y = body_link_waypoints[0][1]
            body_start_point.pose.position.z = body_link_waypoints[0][2]
            body_start_point.pose.orientation.x = body_link_four_elements_list[0][0]
            body_start_point.pose.orientation.y = body_link_four_elements_list[0][1]
            body_start_point.pose.orientation.z = body_link_four_elements_list[0][2]
            body_start_point.pose.orientation.w = body_link_four_elements_list[0][3]

            body_middle_point = PoseStamped()
            body_middle_point.header.frame_id = "body_link"
            body_middle_point.header.stamp = rospy.Time(0)
            body_middle_point.pose.position.x = body_link_waypoints[1][0]
            body_middle_point.pose.position.y = body_link_waypoints[1][1]
            body_middle_point.pose.position.z = body_link_waypoints[1][2]
            body_middle_point.pose.orientation.x = body_link_four_elements_list[1][0]
            body_middle_point.pose.orientation.y = body_link_four_elements_list[1][1]
            body_middle_point.pose.orientation.z = body_link_four_elements_list[1][2]
            body_middle_point.pose.orientation.w = body_link_four_elements_list[1][3]

            body_end_point = PoseStamped()
            body_end_point.header.frame_id = "body_link"
            body_end_point.header.stamp = rospy.Time(0)
            body_end_point.pose.position.x = body_link_waypoints[2][0]
            body_end_point.pose.position.y = body_link_waypoints[2][1]
            body_end_point.pose.position.z = body_link_waypoints[2][2]
            body_end_point.pose.orientation.x = body_link_four_elements_list[2][0]
            body_end_point.pose.orientation.y = body_link_four_elements_list[2][1]
            body_end_point.pose.orientation.z = body_link_four_elements_list[2][2]
            body_end_point.pose.orientation.w = body_link_four_elements_list[2][3]

            #??????body_link????????????????????????????????????????????????
            # body_link_start_rot_message = tf.TransformBroadcaster()
            # body_link_start_rot_message.sendTransform(body_link_waypoints[0], body_link_four_elements_list[0], rospy.Time.now(), "/body_start_loca",
            #                                 "/body_link")
            #
            # body_link_middle_rot_message = tf.TransformBroadcaster()
            # body_link_middle_rot_message.sendTransform(body_link_waypoints[1], body_link_four_elements_list[1],
            #                                  rospy.Time.now(), "/body_middle_loca", "/body_link")
            #
            # body_link_end_rot_message = tf.TransformBroadcaster()
            # body_link_end_rot_message.sendTransform(body_link_waypoints[2], body_link_four_elements_list[2],
            #                               rospy.Time.now(), "/body_end_loca", "/body_link")
            #
            # start_rot_message = tf.TransformBroadcaster()
            # start_rot_message.sendTransform(waypoints[0], four_elements_list[0], rospy.Time.now(), "/start_loca",
            #                                 "/camera2_depth_optical_frame")
            #
            # middle_rot_message = tf.TransformBroadcaster()
            # middle_rot_message.sendTransform([waypoints[1][0], waypoints[1][1], waypoints[1][2]], four_elements_list[1],
            #                                  rospy.Time.now(), "/middle_loca", "/camera2_depth_optical_frame")
            #
            # end_rot_message = tf.TransformBroadcaster()
            # end_rot_message.sendTransform([waypoints[2][0], waypoints[2][1], waypoints[2][2]], four_elements_list[2],
            #                               rospy.Time.now(), "/end_loca", "/camera2_depth_optical_frame")

            Group.set_pose_target(body_start_point, Group_end_effector)
            move_start_plan = Group.plan()


            #body_link???
            if len(move_start_plan.joint_trajectory.points) != 0:
                test_points = self.get_waypoint_orientation(body_link_waypoints, body_link_interpolation_points, body_link_four_elements_list)
                plan = self.move_plan_easy(Group, test_points, Group_end_effector)
                if plan == None:
                    Group.set_joint_value_target(UR10_Q1_to_left)
                    Group.go()
                    return False, None, True
                else:
                    dig_joints_list = self.delete_no_order_joints(plan)

                    new_plan = self.limit_time_toppra(dig_joints_list, 2, ur10_joint_name, False)
                    # print new_plan.joint_trajectory.points[-1]

                    if new_plan != None :
                        rpy_angle = tf.transformations.euler_from_quaternion(body_link_four_elements_list[2])
                        start_angle = rpy_angle[2] * 180 / math.pi
                        # print start_angle
                        inter_angle = (start_angle + (ur10_pull_ready_angle_y - start_angle) / 5) * math.pi / 180
                        # print inter_angle * 180 / math.pi

                        inter_loc_x = body_link_waypoints[2][0] + (ur10_pull_ready_loc[0] - body_link_waypoints[2][0]) / 5
                        inter_loc_y = body_link_waypoints[2][1] + (ur10_pull_ready_loc[1] - body_link_waypoints[2][1]) / 5
                        inter_loc_z = body_link_waypoints[2][2] + 0.15
                        inter_loc = [inter_loc_x, inter_loc_y, inter_loc_z]

                        # qua = self.compute_rotate_matrix([inter_angle], 'z')
                        qua = self.compute_rotate_matrix([inter_angle, 15 * math.pi / 180], 'zy')

                        inter_point = PoseStamped()
                        inter_point.header.frame_id = 'body_link'
                        inter_point.pose.position.x = inter_loc_x
                        inter_point.pose.position.y = inter_loc_y
                        inter_point.pose.position.z = inter_loc_z
                        inter_point.pose.orientation.x = qua[0]
                        inter_point.pose.orientation.y = qua[1]
                        inter_point.pose.orientation.z = qua[2]
                        inter_point.pose.orientation.w = qua[3]

                        dig_time = round(new_plan.joint_trajectory.points[-1].time_from_start.nsecs / 1e9, 3) + \
                                   new_plan.joint_trajectory.points[-1].time_from_start.secs

                        Group.execute(new_plan, wait=True)

                        # dig_start_time = time.time()
                        # end_time = dig_start_time + dig_time - 0.1 #??????????????????????????????
                        # print 'dig_start_time', dig_start_time
                        # print 'end_time', end_time
                        # print 'dig_time', dig_time
                        #
                        # while time.time() <= end_time:
                        #     torque_data = rospy.wait_for_message('/robotiq_ft_wrench', WrenchStamped)
                        #     torque_x = torque_data.wrench.torque.x
                        #     if torque_x >= 10:
                        #         Group.stop()
                        #         print 'stop'
                        #         dig_stop_time = time.time()
                        #         ur10_state = rospy.wait_for_message('/ur10_joint_states', JointState, timeout=1)
                        #         print ur10_state
                        #         print dig_stop_time
                        #         break
                        #
                        # if time.time() < end_time:
                        #     Group.clear_pose_targets()
                        #     rospy.sleep(1)
                        #     read_ur10_current_pose = tf.TransformListener()
                        #     read_ur10_current_pose.waitForTransform("/body_link", "/spoon_body", rospy.Time(),
                        #                                            rospy.Duration(1.0))
                        #
                        #     try:
                        #         (ur10_trans, ur10_rot) = read_ur10_current_pose.lookupTransform("/body_link",
                        #                                                                        "/spoon_body",
                        #                                                                        rospy.Time(0))
                        #         print ur10_trans, ur10_rot
                        #     except:
                        #         pass
                        #     resume_waypoints = [ur10_trans, body_link_waypoints[2]]
                        #     resume_four_elements = [ur10_rot, body_link_four_elements_list[2]]
                        #
                        #     resume_curve = fitting.interpolate_curve(resume_waypoints, 1)
                        #     resume_curve.delta = 0.1
                        #     resume_interpolation_points = resume_curve.evalpts
                        #
                        #     resume_points = self.get_waypoint_orientation(resume_waypoints, resume_interpolation_points, resume_four_elements)
                        #
                        #     resume_plan = self.move_plan_easy(Group, resume_points, Group_end_effector)
                        #     # resume_plan = self.move_plan_easy(Group, [body_link_message.pose, body_end_point.pose], Group_end_effector)
                        #     self.plot_curve(resume_plan)
                        #     Group.execute(resume_plan)

                        rospy.sleep(0.5)

                        for i in range(3):
                            Group.set_pose_target(inter_point, Group_end_effector)
                            move_inter_plan = Group.plan()
                            if len(move_inter_plan.joint_trajectory.points) != 0:
                                can_run = True
                                for i in range(len(move_inter_plan.joint_trajectory.points)):
                                    if -4.5 < move_inter_plan.joint_trajectory.points[i].positions[-1] < -1.74:
                                        pass
                                    else:
                                        can_run = False
                                if can_run == True:
                                    move_inter_plan_success = True
                                    break
                                else:
                                    if i == 2:
                                        move_inter_plan_success = False
                                    else:
                                        pass
                            else:
                                if i == 2:
                                    move_inter_plan_success = False
                                else:
                                    pass

                        if move_inter_plan_success == False:
                            failure_num = 0
                            for i in range(10):
                                print "try {} attempt".format(i)
                                random_delta = np.random.uniform(-0.02, 0.02, 3)
                                inter_point.pose.position.x = inter_point.pose.position.x + random_delta[0]
                                inter_point.pose.position.y = inter_point.pose.position.y + random_delta[1]
                                inter_point.pose.position.z = inter_point.pose.position.z + random_delta[2]
                                Group.set_pose_target(inter_point, Group_end_effector)
                                move_inter_plan = Group.plan()
                                if len(move_inter_plan.joint_trajectory.points) != 0:
                                    can_run = True
                                    for i in range(len(move_inter_plan.joint_trajectory.points)):
                                        if -4.5 < move_inter_plan.joint_trajectory.points[i].positions[-1] < -1.74:
                                            pass
                                        else:
                                            can_run = False
                                    if can_run == True:
                                        break
                                    else:
                                        if i == 9:
                                            return False, None, True
                                        else:
                                            failure_num = failure_num + 1
                                else:
                                    failure_num = failure_num + 1

                            if failure_num == 9:
                                return False, None, True

                        move_dish_joint_list = []
                        move_dish_joint_list.append(move_inter_plan.joint_trajectory.points[0].positions)
                        move_dish_joint_list.append(move_inter_plan.joint_trajectory.points[-1].positions)
                        move_dish_joint_list.append(ur10_pull_ready)

                        move_dish_plan = self.limit_time_toppra(move_dish_joint_list, 2, ur10_joint_name, False)

                        if len(move_dish_plan.joint_trajectory.points) != 0:
                            is_completed = True
                            return is_completed, move_dish_plan, False
                        else:
                            return False, None, True

                    else:
                        is_completed = False
                        return is_completed, None, True

            else:
                Group.set_joint_value_target(UR10_Q1_to_left)
                Group.go()

                is_completed = False
                return is_completed, None, True

        elif action_name == 'initial push':
            push_index = waypoints
            print 'push index', push_index
            file_path = push_file_path

            if push_index == 0:
                push_plan = self.read_tra_message(file_path[0])
                Group.execute(push_plan)
            elif push_index == 1:
                push_plan_1 = self.read_tra_message(file_path[1])
                push_plan_2 = self.read_tra_message(file_path[2])
                Group.execute(push_plan_1)
                Group.execute(push_plan_2)
            elif push_index == 2:
                push_plan = self.read_tra_message(file_path[3])
                Group.execute(push_plan)
            elif push_index == 3:
                push_plan = self.read_tra_message(file_path[4])
                Group.execute(push_plan)


            Group.set_joint_value_target(UR10_Q1_to_left)
            Group.go()
            is_completed = False
            return is_completed



    def read_tra_message_with_up_loc(self, file_path):
        file = open(file_path)
        trajectory_mess = json.load(file)
        # print trajectory_mess
        num_points = len(trajectory_mess)
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = trajectory_mess[0]['joint_names']
        push_up_loc = trajectory_mess[-1]['push_up_loc']
        plan.joint_trajectory.header.frame_id = '/body_link'

        for i in range(1, num_points-1):
            point = JointTrajectoryPoint()
            point.positions = trajectory_mess[i]['position']
            point.velocities = trajectory_mess[i]['velocities']
            point.accelerations = trajectory_mess[i]['accelerations']
            point.time_from_start.secs = trajectory_mess[i]['secs']
            point.time_from_start.nsecs = trajectory_mess[i]['nsecs']
            plan.joint_trajectory.points.append(point)

        return plan, push_up_loc

    def read_tra_message(self, file_path):
        file = open(file_path)
        trajectory_mess = json.load(file)
        # print trajectory_mess
        num_points = len(trajectory_mess)
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = trajectory_mess[0]['joint_names']
        plan.joint_trajectory.header.frame_id = '/body_link'

        for i in range(1, num_points):
            point = JointTrajectoryPoint()
            point.positions = trajectory_mess[i]['position']
            point.velocities = trajectory_mess[i]['velocities']
            point.accelerations = trajectory_mess[i]['accelerations']
            point.time_from_start.secs = trajectory_mess[i]['secs']
            point.time_from_start.nsecs = trajectory_mess[i]['nsecs']
            plan.joint_trajectory.points.append(point)

        return plan

    def ur10_ik_kinematics(self, body_link_mess):
        transform_ur10_base_link = tf.TransformListener()
        num = len(body_link_mess)
        joints_list = []
        for i in range(num):
            ur10_base_link_mess = transform_ur10_base_link.transformPose('/ur10_base_link', body_link_mess[i])
            matrix = tf.transformations.quaternion_matrix([ur10_base_link_mess.pose.orientation.x, ur10_base_link_mess.pose.orientation.y,
                                                           ur10_base_link_mess.pose.orientation.z, ur10_base_link_mess.pose.orientation.w])

            matrix[0][3] = ur10_base_link_mess.pose.position.x
            matrix[1][3] = ur10_base_link_mess.pose.position.y
            matrix[2][3] = ur10_base_link_mess.pose.position.z

            # print matrix
            robot = URDF.from_parameter_server()
            kdl_kin = KDLKinematics(robot, "ur10_base_link", "spoon_body")

            ik = kdl_kin.inverse(matrix, maxiter=1000)
            joints_list.append(ik)
        return joints_list

    def get_rotate_angle_to_end(self, start_loc, end_loc):
        delta_x = end_loc[0] - start_loc[0]
        delta_y = end_loc[1] - start_loc[1]
        delta_z = end_loc[2] - start_loc[2]

        angle_x = math.atan2(delta_y, delta_z)
        y_z = math.sqrt(pow(delta_z, 2) + pow(delta_y, 2))
        angle_y = math.atan2(delta_x, y_z)

        return -angle_x, angle_y

    def compute_rotate_location(self, angle_x, angle_y, angle_z, location):
        matrix_z = [[math.cos(angle_z), -math.sin(angle_z), 0, 0],
                    [math.sin(angle_z), math.cos(angle_z), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        matrix_x = [[1, 0, 0, 0], [0, math.cos(angle_x), -math.sin(angle_x), 0],
                    [0, math.sin(angle_x), math.cos(angle_x), 0], [0, 0, 0, 1]]
        matrix_y = [[math.cos(angle_y), 0, math.sin(angle_y), 0], [0, 1, 0, 0],
                    [-math.sin(angle_y), 0, math.cos(angle_y), 0], [0, 0, 0, 1]]


        matrix_temp = numpy.dot(matrix_z, matrix_y)
        matrix = numpy.dot(matrix_temp, matrix_x)
        input_location = numpy.dot(matrix, location)

        return input_location

    def get_push_rotate_angle(self, start_loc, end_loc):
        delta_x = abs(end_loc[0] - start_loc[0])
        delta_y = abs(end_loc[1] - start_loc[1])

        theta = math.atan2(delta_x, delta_y)
        return theta

    def compute_rotated_angle_y(self, input_loc, angle_list, rotate_order):
        if len(angle_list) == len(rotate_order):
            matrix = [[1,0,0], [0,1,0], [0,0,1]]

            index = 0
            for s in rotate_order:
                if s == 'x':
                    angle_x = angle_list[index]
                    matrix_x = [[1, 0, 0], [0, math.cos(angle_x), -math.sin(angle_x)],
                                [0, math.sin(angle_x), math.cos(angle_x)]]
                    matrix = numpy.dot(matrix, matrix_x)
                    index += 1
                elif s == 'y':
                    angle_y = angle_list[index]
                    matrix_y = [[math.cos(angle_y), 0, math.sin(angle_y)], [0, 1, 0],
                                [-math.sin(angle_y), 0, math.cos(angle_y)]]
                    matrix = numpy.dot(matrix, matrix_y)
                    index += 1
                elif s == 'z':
                    angle_z = angle_list[index]
                    matrix_z = [[math.cos(angle_z), -math.sin(angle_z), 0],
                                [math.sin(angle_z), math.cos(angle_z), 0], [0, 0, 1]]
                    matrix = numpy.dot(matrix, matrix_z)
                    index += 1
            input_loc = numpy.asarray(input_loc)
            rotate_xyz = numpy.dot(matrix, input_loc.T)
            rotated_angle_y = math.atan2(rotate_xyz[2], math.sqrt(pow(rotate_xyz[0], 2) + pow(rotate_xyz[1], 2)))
        else:
            print "input error"
        return rotated_angle_y

    def compute_rotate_matrix(self, angle_list, rotate_order):
        if len(angle_list) == len(rotate_order):
            matrix = [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]

            index = 0
            for s in rotate_order:
                if s == 'x':
                    angle_x = angle_list[index]
                    matrix_x = [[1, 0, 0, 0], [0, math.cos(angle_x), -math.sin(angle_x), 0],
                                [0, math.sin(angle_x), math.cos(angle_x), 0], [0, 0, 0, 1]]
                    matrix = numpy.dot(matrix, matrix_x)
                    index += 1
                elif s == 'y':
                    angle_y = angle_list[index]
                    matrix_y = [[math.cos(angle_y), 0, math.sin(angle_y), 0], [0, 1, 0, 0],
                                [-math.sin(angle_y), 0, math.cos(angle_y), 0], [0, 0, 0, 1]]
                    matrix = numpy.dot(matrix, matrix_y)
                    index += 1
                elif s == 'z':
                    angle_z = angle_list[index]
                    matrix_z = [[math.cos(angle_z), -math.sin(angle_z), 0, 0],
                                [math.sin(angle_z), math.cos(angle_z), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
                    matrix = numpy.dot(matrix, matrix_z)
                    index += 1
            input_quaternion = tf.transformations.quaternion_from_matrix(matrix)

        else:
            print "input error"
        return input_quaternion


    def get_waypoint_orientation(self, waypoints,  non_orientation_waypoint, four_elememts_list):
        # ??????????????????
        m = len(waypoints)
        # ??????????????????
        n = len(non_orientation_waypoint)
        get_orientation_waypoint = []

        temp_point = PoseStamped()
        temp_point.header.frame_id = 'body_link'

        # ??????????????????
        for i in range(m - 1):
            pose_temp = []
            # print (i)
            pose_temp = self.slerp(four_elememts_list[i], four_elememts_list[i + 1], n / (m - 1))
            # print pose_temp
            for j in range(n / (m - 1)):
                temp_point.pose.position.x = non_orientation_waypoint[j + i * n / (m - 1)][0]
                temp_point.pose.position.y = non_orientation_waypoint[j + i * n / (m - 1)][1]
                temp_point.pose.position.z = non_orientation_waypoint[j + i * n / (m - 1)][2]
                temp_point.pose.orientation.x = pose_temp[j][0]
                temp_point.pose.orientation.y = pose_temp[j][1]
                temp_point.pose.orientation.z = pose_temp[j][2]
                temp_point.pose.orientation.w = pose_temp[j][3]
                tempose = deepcopy(temp_point.pose)
                get_orientation_waypoint.append(deepcopy(tempose))

        # ????????????????????????
        # pose_temp = self.slerp(four_elememts_list[0], four_elememts_list[2], 100)
        # for i in range(n):
        #     temp_point.pose.position.x = non_orientation_waypoint[i][0]
        #     temp_point.pose.position.y = non_orientation_waypoint[i][1]
        #     temp_point.pose.position.z = non_orientation_waypoint[i][2]
        #     temp_point.pose.orientation.x = pose_temp[i][0]
        #     temp_point.pose.orientation.y = pose_temp[i][1]
        #     temp_point.pose.orientation.z = pose_temp[i][2]
        #     temp_point.pose.orientation.w = pose_temp[i][3]
        #     tempose = deepcopy(temp_point.pose)
        #     get_orientation_waypoint.append(deepcopy(tempose))

        # print (get_orientation_waypoint)

        # ????????????
        # pose_all = self.slerp(original_points[0].pose.orientation, original_points[2].pose.orientation, 100)

        return get_orientation_waypoint


    def slerp(self, start_orientation, end_orientation, num):

        target_orientation = []

        # start_normalize_orientation = self.normalize_orientation(start_orientation)
        # end_normalize_orientation = self.normalize_orientation(end_orientation)
        # print (start_normalize_orientation, end_normalize_orientation)

        # start_normalize_orientation = self.orientation_to_list(start_orientation)
        # end_normalize_orientation = self.orientation_to_list(end_orientation)
        # print (start_normalize_orientation, end_normalize_orientation)

        cos = numpy.dot(start_orientation, end_orientation)

        if (cos < 0):
            for i in range(len(start_orientation)):
                end_orientation[i] = - end_orientation[i]
            cos = -cos
        if (cos > 0.995):
            for i in range(num):
                target_orientation.append(start_orientation)
            return target_orientation

        angle = math.acos(cos)
        for i in range(num):
            t = 1 - i * 1.0 / (num - 1)
            a = math.sin(angle * t) / math.sin(angle)
            b = math.sin(angle * i * 1.0 / (num - 1)) / math.sin(angle)
            change_matrix = []
            for j in range(4):
                change_matrix.append(a * start_orientation[j] + b * end_orientation[j])

            target_orientation.append(change_matrix)

        return target_orientation

    def move_plan_easy(self, Group, waypoint_test, Group_end_effector):
        fraction = 0.0  # ?????????????????????
        maxtries = 10  # ????????????????????????
        attempts = 0  # ????????????????????????

        # ??????????????????????????????????????????????????????
        Group.set_start_state_to_current_state()

        # ?????????????????????????????????????????????????????????????????????????????????????????????
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = Group.compute_cartesian_path(
                waypoint_test,  # waypoint poses???????????????
                0.01,  # eef_step??????????????????
                0.0,  # jump_threshold???????????????
                True)  # avoid_collisions???????????????

            # print fraction
            # ??????????????????
            attempts += 1

            # ????????????????????????
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # ????????????????????????????????????100%???,??????????????????????????????
        if (fraction == 1.0):
            rospy.loginfo("Path computed successfully. ")
            # print plan
            return plan
        # ????????????????????????????????????????????????
        else:
            rospy.loginfo(
                "Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

    def delete_no_order_joints(self, plan):
        num = len(plan.joint_trajectory.points)
        # print num
        pos = np.ones((6, num))

        for i in range(num):
            for j in range(6):
                pos[j][i] = plan.joint_trajectory.points[i].positions[j]

        # for i in range(num-2, -1, -1):
        #     if i % 2 == 1:
        #         pos = np.delete(pos, i, axis=1)
        # print pos
        new_num = pos.shape[1]
        trend = []
        for i in range(6):
            if pos[i][0] - pos[i][-1] > 0:
                trend.append(-1)
            else:
                trend.append(1)


        delete_num = []
        for i in range(6):
            if trend[i] == -1:
                for j in range(new_num-1):
                    if pos[i][j] < pos[i][j+1]:
                        delete_num.append(j+1)
                    # if pos[i][j] - pos[i][j+1] < 0.0004:
                    #     delete_num.append(j + 1)

            else:
                for j in range(new_num-1):
                    if pos[i][j] > pos[i][j+1]:
                        delete_num.append(j+1)
                    # if pos[i][j+1] - pos[i][j] < 0.0004:
                    #     delete_num.append(j + 1)
        delete_num = np.unique(delete_num)
        # print delete_num
        # for i in reversed(delete_num):
        #     pos = np.delete(pos, i, axis=1)
        # print pos
        return pos.T

    def limit_time_toppra(self, pos, move_time, joint_name, is_show):
        num = len(pos)
        # print pos
        tt = np.linspace(0, 1, num)
        # print len(tt)
        # print len(pos)
        path = ta.SplineInterpolator(tt, pos)

        # vlim_ = np.asarray([3.5, 3.5, 3.5, 3.5, 3.5, 4])
        vlim_ = np.asarray([3.5, 3.5, 3.5, 4, 4, 4])
        vlim = np.vstack((-vlim_, vlim_)).T
        # alim_ = np.asarray([3.5, 2.5, 2.5, 2.5, 2.5, 6.6])
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
        print 'time', jnt_traj.duration
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
