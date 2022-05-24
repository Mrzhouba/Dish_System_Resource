#!/usr/bin/env python
#coding=UTF-8
import math
import rospy
import numpy
import copy
# import roslib; roslib.load_manifest('ur_driver')
import moveit_msgs.msg
import roslib
import math
# roslib.load_manifest('ur_driver')
# roslib.load_manifest('dh_hand_driver')

from robot_order.msg import food_msg

import json

from copy import deepcopy
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from geomdl import fitting
from geomdl import BSpline
from geomdl import exchange
from geomdl.visualization import VisMPL
from moveit_msgs.msg import RobotTrajectory
from control_msgs.msg import *
from trajectory_msgs.msg import *


class trajectory_plan:

    def B_curve_plan(self, Group, waypoint, move_time):
        end_points = self.get_position_matrix(waypoint)
        # print end_points
        # curve = BSpline.Curve()
        # # 设置曲线次数k
        # curve.degree = 2
        # # 设置控制点
        # curve.ctrlpts = end_points
        # # 设置节点矢量
        # curve.knotvector = [0, 0, 0, 1, 1, 1]
        # # 设置递增步距，生成总点数为1/0.05=20
        # curve.delta = 0.01


        # Do global curve interpolation
        n = len(waypoint)
        if (n == 2):
            degree = 1
        else:
            degree = 2
        # print end_points
        curve = fitting.interpolate_curve(end_points, degree)
        curve.delta = 0.1
        points_2 = curve.evalpts
        print len(points_2)
        test_points = self.get_waypoint_orientation(waypoint, points_2)
        # test_points = test_points[::2]
        # reduce_test_points = []
        # reduce_test_points.append(test_points[0])
        # for i in range(49,52):
        #     reduce_test_points.append(test_points[i])
        #
        # reduce_test_points.append(test_points[-1])
        # print (len(test_points))
        # print len(reduce_test_points)
        # print reduce_test_points

        no_limit_plan = self.move_plan_easy(Group, test_points)

        # plan = self.three_limit_time_plan(no_limit_plan, move_time)
        # plan = self.realtime(no_limit_plan, move_time)
        # plan = self.three_B_limit_time_plan(no_limit_plan, 5)
        # plan = self.average_vel_add_three(no_limit_plan, move_time)
        return test_points, no_limit_plan

        # ur3_test_plan = self.move_plan_easy(ur3_arm,points_2)
        # ur3_arm.plan(points_2)

        # Load control points from a text file
        # curve.ctrlpts = exchange.import_txt("/home/cqy327/catkin_ws/src/ur3ur10_modern_driver/control_points.txt")

        # Export the curve as a JSON file
        # exchange.export_json(curve, "curve.json")
        # curve.shape.data.

        # Plot the interpolated curve
        # curve.delta = 0.01
        # curve.vis = VisMPL.VisCurve3D()
        # curve.render()

    def get_position_matrix(self, waypoints):
        n = len(waypoints)
        # print n
        all_points = []
        # print all_points

        for i in range(n):
            points = []
            points.append(waypoints[i].pose.position.x)
            points.append(waypoints[i].pose.position.y)
            points.append(waypoints[i].pose.position.z)

            if (i == 0):
                all_points = points
            else:
                all_points = numpy.append(all_points, points)
        # print all_points
        end_points = numpy.array(all_points).reshape(n, 3).tolist()
        return end_points

    def get_waypoint_orientation(self, waypoints, non_orientation_waypoint):
        # 初始点的数量
        m = len(waypoints)
        # 插补点的数量
        n = len(non_orientation_waypoint)
        # print m, n
        # print waypoints
        get_orientation_waypoint = []

        original_points = waypoints

        temp_point = PoseStamped()
        temp_point.header.frame_id = 'body_link'

        # print non_orientation_waypoint
        # 位置插补
        # for j in range (m):
        #     pose_all = []
        #     for i in range(n/m):

        #         temp_point.header.stamp = rospy.Time.now()
        #         temp_point.pose.position.x = non_orientation_waypoint[i + j*n/m][0]
        #         temp_point.pose.position.y = non_orientation_waypoint[i + j*n/m][1]
        #         temp_point.pose.position.z = non_orientation_waypoint[i + j*n/m][2]
        #         temp_point.pose.orientation.x = original_points[j].pose.orientation.x
        #         temp_point.pose.orientation.y = original_points[j].pose.orientation.y
        #         temp_point.pose.orientation.z = original_points[j].pose.orientation.z
        #         temp_point.pose.orientation.w = original_points[j].pose.orientation.w
        #         tempose = deepcopy(temp_point.pose)
        #         get_orientation_waypoint.append(deepcopy(tempose))
        # pose_temp = self.slerp(original_points[0].pose.orientation, original_points[m - 1].pose.orientation, n)
        # print len(pose_temp)
        # print pose_temp

        # 位置姿态插补
        for i in range(m - 1):
            pose_temp = []
            # print (i)
            pose_temp = self.slerp(original_points[i].pose.orientation, original_points[i + 1].pose.orientation,
                                   n / (m - 1))
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

        # 起始末尾两个姿态
        # pose_temp = self.slerp(original_points[0].pose.orientation, original_points[m - 1].pose.orientation, n)
        # # print pose_temp
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

        # 姿态插补
        # pose_all = self.slerp(original_points[0].pose.orientation, original_points[2].pose.orientation, 100)

        return get_orientation_waypoint

    def move_plan_easy(self, Group, waypoint_test):
        # print len(waypoint_test)
        # print (waypoint_test)
        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        Group.set_start_state_to_current_state()

        # print both_waypoints[1]
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = Group.compute_cartesian_path(
                waypoint_test,  # waypoint poses，路点列表
                0.01,  # eef_step，终端步进值
                0.0,  # jump_threshold，跳跃阈值
                True)  # avoid_collisions，避障规划

            # print fraction
            # 尝试次数累加
            attempts += 1

            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if (fraction == 1.0):
            rospy.loginfo("Path computed successfully. ")
            # print plan
            return plan
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo(
                "Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

    def allot_time(self, plan, move_time):
        length = len(plan.joint_trajectory.points)
        allot_time = []
        six_angle_dis = []
        alloted_time_per = []
        alloted_time_per.append(1)
        index = 0

        for i in range(6):
            dis = abs(plan.joint_trajectory.points[length-1].positions[i] - plan.joint_trajectory.points[0].positions[i]) *1.0
            six_angle_dis.append(dis)

        for i in range(length-2):
            max_dis = 0
            for j in range(6):
                distance = abs(plan.joint_trajectory.points[i+1].positions[j] - plan.joint_trajectory.points[i].positions[j])
                if distance > max_dis:
                    max_dis = distance
                    index = j
            # max_dis = abs(plan.joint_trajectory.points[i + 1].positions[0] - plan.joint_trajectory.points[i].positions[0])
            dis_percentage = max_dis * 1.0 / six_angle_dis[index]
            if(0.015 < dis_percentage < 0.05):
                alloted_time_per.append(2)
            elif(dis_percentage > 0.05):
                alloted_time_per.append(5)
            else:
                alloted_time_per.append(1)
        total = 0
        per_time = 0
        for i in range(len(alloted_time_per)):
            total += alloted_time_per[i]
        print alloted_time_per
        print total
        time = move_time * 1.0 / total
        for i in range(len(alloted_time_per)):
            per_time += alloted_time_per[i] * time
            allot_time.append(per_time)
        return allot_time

    def realtime(self, plan, time):
        new_plan = copy.deepcopy(plan)
        num = len(plan.joint_trajectory.points)
        every_time = time*1.0 / (num-1)
        print every_time

        for i in range(num):
            new_plan.joint_trajectory.points[i].velocities = [0, 0, 0, 0, 0, 0]
            if i == 0:
                new_plan.joint_trajectory.points[0].velocities = [0,0,0,0,0,0]

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * every_time)[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * every_time)[0] * 1000000000
            elif i == (num-1):
                new_plan.joint_trajectory.points[num-1].velocities = [0,0,0,0,0,0]

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * every_time)[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * every_time)[0] * 1000000000
            else:
                for j in range(6):
                    new_velocity = (plan.joint_trajectory.points[i+1].positions[j] - plan.joint_trajectory.points[i-1].positions[j]) *1.0 / (2*every_time)

                    new_plan.joint_trajectory.points[i].velocities[j] = new_velocity

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * every_time)[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * every_time)[0] * 1000000000

        for i in range(num-1):
            new_plan.joint_trajectory.points[i].accelerations = [0, 0, 0, 0, 0, 0]
            if i == (num-1):
                new_plan.joint_trajectory.points[num - 1].accelerations = [0, 0, 0, 0, 0, 0]
            if(i == 1 or i == 2):
                for j in range(6):
                    new_acc = (new_plan.joint_trajectory.points[i+1].velocities[j] - new_plan.joint_trajectory.points[i].velocities[j]) *1.0 /(2*every_time)
                    # print new_acc
                    new_plan.joint_trajectory.points[i].accelerations[j] = new_acc
            else:
                for j in range(6):
                    new_acc = (new_plan.joint_trajectory.points[i+1].velocities[j] - new_plan.joint_trajectory.points[i].velocities[j]) *1.0 /every_time
                    # print new_acc
                    new_plan.joint_trajectory.points[i].accelerations[j] = new_acc

        # print new_plan
        return new_plan

    def change_realtime(self, plan, time):
        allot_time = self.allot_time(plan, time)
        print allot_time
        print len(allot_time)
        new_plan = copy.deepcopy(plan)
        num = len(plan.joint_trajectory.points)
        every_time = time*1.0 / (num-1)

        for i in range(num-2):
            new_plan.joint_trajectory.points[i].velocities = [0, 0, 0, 0, 0, 0]
            if i == 0:
                new_plan.joint_trajectory.points[0].velocities = [0,0,0,0,0,0]

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * every_time)[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * every_time)[0] * 1000000000
            elif i == (num-2):
                new_plan.joint_trajectory.points[num-1].velocities = [0,0,0,0,0,0]

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(allot_time[num-2])[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(allot_time[num-2])[0] * 1000000000
            elif (i == 1 or i == 2):
                for j in range(6):
                    new_velocity = (plan.joint_trajectory.points[i+1].positions[j] - plan.joint_trajectory.points[i-1].positions[j]) *1.0 / 2*(allot_time[i] - allot_time[i-1])

                    new_plan.joint_trajectory.points[i].velocities[j] = new_velocity

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(allot_time[i])[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(allot_time[i])[0] * 1000000000
            else:
                for j in range(6):
                    new_velocity = (plan.joint_trajectory.points[i+1].positions[j] - plan.joint_trajectory.points[i-1].positions[j]) *1.0 / (allot_time[i+1] - allot_time[i-1])

                    new_plan.joint_trajectory.points[i].velocities[j] = new_velocity

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(allot_time[i])[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(allot_time[i])[0] * 1000000000

        for i in range(num):
            new_plan.joint_trajectory.points[i].accelerations = [0, 0, 0, 0, 0, 0]
            if (i == (num-1) or i == 0):
                new_plan.joint_trajectory.points[num - 1].accelerations = [0, 0, 0, 0, 0, 0]
            elif (i == 1 or i == 2):
                for j in range(6):
                    new_acc = (new_plan.joint_trajectory.points[i + 1].velocities[j] -
                               new_plan.joint_trajectory.points[i].velocities[j]) * 1.0 / 2*(allot_time[i] - allot_time[i-1])
                    # print new_acc
                    new_plan.joint_trajectory.points[i].accelerations[j] = new_acc
            else:
                for j in range(6):
                    new_acc = (new_plan.joint_trajectory.points[i+1].velocities[j] - new_plan.joint_trajectory.points[i].velocities[j]) *1.0 /(allot_time[i] - allot_time[i-1])
                    # print new_acc
                    new_plan.joint_trajectory.points[i].accelerations[j] = new_acc

        # print new_plan
        return new_plan
    def three_B_limit_time_plan(self, plan, move_time):
        new_plan  = copy.deepcopy(plan)
        nums = len(plan.joint_trajectory.points)
        t = move_time * 1.0 / nums
        for i in range(nums):
            new_plan.joint_trajectory.points[i].positions = [0,0,0,0,0,0]
            new_plan.joint_trajectory.points[i].velocities = [0,0,0,0,0,0]
            new_plan.joint_trajectory.points[i].accelerations = [0,0,0,0,0,0]

            new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * t)[1]
            new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * t)[0] * 1000000000
        for i in range(0, 99, 3):
            # print i
            # plan.joint_trajectory.points[i].positions = [0,0,0,0,0,0]
            # plan.joint_trajectory.points[i].velocities = [0,0,0,0,0,0]
            # plan.joint_trajectory.points[i].accelerations = [0,0,0,0,0,0]
            for j in range(6):
                new_joint_state, new_velocity, new_acc = self.compute_every_curve_B(plan.joint_trajectory.points[i].positions[j], plan.joint_trajectory.points[i+1].positions[j], plan.joint_trajectory.points[i+2].positions[j], plan.joint_trajectory.points[i+3].positions[j], 4*t)

                for k in range(4):

                    new_plan.joint_trajectory.points[i+k].positions[j] = new_joint_state[k]
                    new_plan.joint_trajectory.points[i+k].velocities[j] = new_velocity[k]
                    new_plan.joint_trajectory.points[i+k].accelerations[j] = new_acc[k]



        return new_plan

    def compute_every_curve_B(self, x1, x2, x3, x4, time):

        V1 = (2*x1 + 2*x2 + 7*x4 - x3) * 1.0 / 6 - x4
        V3 = (2*x1 + 2*x2 + 7*x4 - x3) * 1.0 / 6
        V2 = (x3 + x4 - 2*V3) * 1.0 / 4
        V0 = V2 - x1

        t = time * 1.0 / 4
        new_joint_state = []
        new_velocity = []
        new_acc = []
        for i in range(4):
            joint_state = ((1-3*i*t+3*pow(i*t,2)-pow(i*t,3)) * V0 +(4+3*pow(i*t, 3) - 6*pow(i*t,2)) * V1 + (1+3*i*t+3*pow(i*t,2)-3*pow(i*t,3)) *V2 + pow(i*t,3)* V3) * 1.0 / 6
            velocity = (-pow((1-i*t), 2)*V0 + (3*pow(i*t, 3) - 4*i*t)*V1 + (-3*pow(i*t,2)+2*i*t+1)*V2 + pow(i*t, 2)*V3) * 1.0 / 2
            acc = (1-i*t)*V0 + (3*i*t - 2)*V1 + (1-3*i*t)*V2 + i*t*V3
            new_joint_state.append(joint_state)
            new_velocity.append(velocity)
            new_acc.append(acc)
        # print new_acc
        return new_joint_state, new_velocity, new_acc

    def average_vel_add_three(self, plan, time):
        new_plan = copy.deepcopy(plan)
        num = len(plan.joint_trajectory.points)
        every_time = time * 1.0 / (num - 1)
        # print every_time

        for i in range(num):
            new_plan.joint_trajectory.points[i].velocities = [0, 0, 0, 0, 0, 0]
            if i == 0:
                new_plan.joint_trajectory.points[0].velocities = [0, 0, 0, 0, 0, 0]

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * every_time)[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * every_time)[0] * 1000000000
            elif i == (num - 1):
                new_plan.joint_trajectory.points[num - 1].velocities = [0, 0, 0, 0, 0, 0]

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * every_time)[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * every_time)[0] * 1000000000
            else:
                for j in range(6):
                    new_velocity = (plan.joint_trajectory.points[i + 1].positions[j] -
                                    plan.joint_trajectory.points[i - 1].positions[j]) * 1.0 / (2 * every_time)

                    new_plan.joint_trajectory.points[i].velocities[j] = new_velocity

                new_plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * every_time)[1]
                new_plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * every_time)[0] * 1000000000

        for i in range(num-1):
            new_plan.joint_trajectory.points[i].accelerations = [0, 0, 0, 0, 0, 0]
            for j in range(6):
                new_joint_state, new_velocity, new_acc = self.three_cubic(new_plan.joint_trajectory.points[i].positions[j], new_plan.joint_trajectory.points[i+1].positions[j],
                                                                          new_plan.joint_trajectory.points[i].velocities[j], new_plan.joint_trajectory.points[i+1].velocities[j], every_time)
                new_plan.joint_trajectory.points[i].accelerations[j] = new_acc

        return new_plan

    def three_cubic(self, start_pos, end_pos, start_vel, end_vel, time):
        a0 = start_pos
        a1 = start_vel
        a2 = (3 * (end_pos - start_pos) - time * (2 * start_vel + end_vel)) / pow(time,2)
        a3 = (-2 * (end_pos - start_pos) + time * (start_vel + end_vel)) / pow(time,3)

        t = time * 1.0 / 5
        for i in range(5):
            new_joint_state = a0 + a1 * i*t + a2 * pow(i*t, 2) + a3 * pow(i*t, 3)
            new_velocity = a1 + 2 * a2 * i*t + 3 * a3 * pow(i*t, 2)
            new_acc = 2 * a2 + 6 * a3 * i*t

        return new_joint_state, new_velocity, new_acc

    def three_limit_time_plan(self, plan, move_time):

        nums = len(plan.joint_trajectory.points)
        t = move_time*1.0 / nums

        start_joint_state = plan.joint_trajectory.points[0].positions
        end_joint_state = plan.joint_trajectory.points[nums-1].positions
        start_velocity = [0,0,0,0,0,0]
        end_velocity = [0,0,0,0,0,0]

        a0 = start_joint_state
        a1 = start_velocity
        a2 = [0,0,0,0,0,0]
        a3 = [0,0,0,0,0,0]

        for i in range(6):
            a2[i] = (3 * (end_joint_state[i] - start_joint_state[i]) - move_time * (2 * start_velocity[i] + end_velocity[i]))/ pow(move_time,2)
            a3[i] = (-2 * (end_joint_state[i] - start_joint_state[i]) + move_time * (start_velocity[i] + end_velocity[i]))/ pow(move_time,3)

        for i in range(nums):
            plan.joint_trajectory.points[i].positions = [0,0,0,0,0,0]
            plan.joint_trajectory.points[i].velocities = [0,0,0,0,0,0]
            plan.joint_trajectory.points[i].accelerations = [0,0,0,0,0,0]
            for j in range(6):
                new_joint_state = a0[j] + a1[j] * i*t + a2[j] * pow(i*t,2) + a3[j] * pow(i*t,3)
                new_velocity = a1[j] + 2 * a2[j] * i*t + 3 * a3[j] * pow(i*t,2)
                new_acc = 2 * a2[j] + 6 * a3[j] * i*t
                plan.joint_trajectory.points[i].positions[j] = new_joint_state
                plan.joint_trajectory.points[i].velocities[j] = new_velocity
                plan.joint_trajectory.points[i].accelerations[j] = new_acc

            plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * t)[1]
            plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * t)[0]*1000000000

        return plan

    def five_limit_time_plan(self, plan, move_time):
        nums = len(plan.joint_trajectory.points)
        t = move_time * 1.0 / nums

        start_joint_state = plan.joint_trajectory.points[0].positions
        end_joint_state = plan.joint_trajectory.points[nums - 1].positions
        print start_joint_state
        print end_joint_state
        start_velocity = [0, 0, 0, 0, 0, 0]
        end_velocity = [0, 0, 0, 0, 0, 0]
        start_acc = [0, 0, 0, 0, 0, 0]
        end_acc = [0, 0, 0, 0, 0, 0]

        a0 = start_joint_state
        a1 = start_velocity
        a2 = [0, 0, 0, 0, 0, 0]
        a3 = [0, 0, 0, 0, 0, 0]
        a4 = [0, 0, 0, 0, 0, 0]
        a5 = [0, 0, 0, 0, 0, 0]

        for i in range(6):
            a3[i] = 1.0 / (2 * pow(move_time, 3)) * (20 * (end_joint_state[i] - start_joint_state[i]) - (8*end_velocity[i] + 12*start_velocity[i])*move_time + (end_acc[i] - 3*start_acc[i]) / pow(move_time,2))
            a4[i] = 1.0/(2*pow(move_time,4)) * (-30*(end_joint_state[i] - start_joint_state[i]) + (14*end_velocity[i] + 16*start_velocity[i]) * move_time + (3*start_acc[i]-2*end_acc[i])/pow(move_time,2))
            a5[i] = 1.0/(2*pow(move_time,5)) * (12*(end_joint_state[i] - start_joint_state[i]) - 6*(start_velocity[i] + end_velocity[i])*move_time + (end_acc[i] - start_acc[i])/pow(move_time,2))

        for i in range(nums):
            plan.joint_trajectory.points[i].positions = [0,0,0,0,0,0]
            plan.joint_trajectory.points[i].velocities = [0,0,0,0,0,0]
            plan.joint_trajectory.points[i].accelerations = [0,0,0,0,0,0]
            for j in range(6):
                new_joint_state = a0[j] + a1[j] * i*t + a2[j] * pow(i*t,2) + a3[j] * pow(i*t,3) + a4[j]*pow(i*t, 4) + a5[j]*pow(i*t, 5)
                new_velocity = a1[j] + 2 * a2[j] * i*t + 3 * a3[j] * pow(i*t,2) + 4*a4[j]*pow(i*t, 3) + 5*a5[j]*pow(i*t, 4)
                new_acc = 2 * a2[j] + 6 * a3[j] * i*t + 12*a4[j]*pow(i*t,2) + 20*a5[j]*pow(i*t, 3)
                plan.joint_trajectory.points[i].positions[j] = new_joint_state
                plan.joint_trajectory.points[i].velocities[j] = new_velocity
                plan.joint_trajectory.points[i].accelerations[j] = new_acc

            plan.joint_trajectory.points[i].time_from_start.secs = math.modf(i * t)[1]
            plan.joint_trajectory.points[i].time_from_start.nsecs = math.modf(i * t)[0]*1000000000

        return plan




    def normalize_orientation(self, source_orientation):
        temp = []
        temp.append(source_orientation.x)
        temp.append(source_orientation.y)
        temp.append(source_orientation.z)
        temp.append(source_orientation.w)

        target_orientation = 0
        for i in temp:
            target_orientation += i * i
            print i
            print (target_orientation)
        target_orientation = math.sqrt(target_orientation)
        # print ("123")
        print (target_orientation)
        return target_orientation

    def orientation_to_list(self, orientation):
        temp = []
        temp.append(orientation.x)
        temp.append(orientation.y)
        temp.append(orientation.z)
        temp.append(orientation.w)

        return temp

    def slerp(self, start_orientation, end_orientation, num):

        target_orientation = []

        # start_normalize_orientation = self.normalize_orientation(start_orientation)
        # end_normalize_orientation = self.normalize_orientation(end_orientation)
        # print (start_normalize_orientation, end_normalize_orientation)

        start_normalize_orientation = self.orientation_to_list(start_orientation)
        end_normalize_orientation = self.orientation_to_list(end_orientation)
        # print (start_normalize_orientation, end_normalize_orientation)

        cos = numpy.dot(start_normalize_orientation, end_normalize_orientation)
        # print (cos)
        if (cos < 0):
            for i in range(len(start_normalize_orientation)):
                end_normalize_orientation[i] = - end_normalize_orientation[i]
            # print (start_normalize_orientation)
            cos = -cos
        if (cos > 0.95):
            for i in range(num):
                target_orientation.append(start_normalize_orientation)
                # print (target_orientation)
            return target_orientation

        angle = math.acos(cos)
        for i in range(num):
            t = 1 - i * 1.0 / (num - 1)
            a = math.sin(angle * t) / math.sin(angle)
            b = math.sin(angle * i * 1.0 / (num - 1)) / math.sin(angle)
            # print (i,angle,t,a,b)
            change_matrix = []
            for j in range(4):
                change_matrix.append(a * start_normalize_orientation[j] + b * end_normalize_orientation[j])

            target_orientation.append(change_matrix)

        # print (target_orientation)
        return target_orientation

    def move_plan(self, start_pose, Group, length, intervalNum):

        Get_waypoint = self.circle_move(start_pose, length, intervalNum, 1)

        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        Group.set_start_state_to_current_state()

        # print both_waypoints[1]
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = Group.compute_cartesian_path(
                Get_waypoint,  # waypoint poses，路点列表
                0.01,  # eef_step，终端步进值
                0.0,  # jump_threshold，跳跃阈值
                True)  # avoid_collisions，避障规划

            # 尝试次数累加
            attempts += 1

            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if (fraction == 1.0):
            rospy.loginfo("Path computed successfully. ")
            return plan
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo(
                "Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

    def circle_move(self, start_move_pose, radius, interval, rotation):
        # rotation == 1 为xy方向; rotation == 2 为xz方向; rotation == 3 为yz方向
        # 初始化路点列表
        waypoint = []

        # 将圆弧上的路径点加入列表
        waypoint.append(start_move_pose.pose)

        if rotation == 1:
            centerA = start_move_pose.pose.position.x
            centerB = start_move_pose.pose.position.y

            for th in numpy.arange(0, 6.28, interval):
                start_move_pose.pose.position.x = centerA + radius * math.cos(th)
                start_move_pose.pose.position.y = centerB + radius * math.sin(th)
                wpose = deepcopy(start_move_pose.pose)
                waypoint.append(deepcopy(wpose))

            return waypoint

        if rotation == 2:
            centerA = start_move_pose.pose.position.x
            centerB = start_move_pose.pose.position.z

            for th in numpy.arange(0, 6.28, interval):
                start_move_pose.pose.position.x = centerA + radius * math.cos(th)
                start_move_pose.pose.position.z = centerB + radius * math.sin(th)
                wpose = deepcopy(start_move_pose.pose)
                waypoint.append(deepcopy(wpose))

            return waypoint

        if rotation == 3:
            centerA = start_move_pose.pose.position.y
            centerB = start_move_pose.pose.position.z

            for th in numpy.arange(0, 6.28, interval):
                start_move_pose.pose.position.x = centerA + radius * math.cos(th)
                start_move_pose.pose.position.y = centerB + radius * math.sin(th)
                wpose = deepcopy(start_move_pose.pose)
                waypoint.append(deepcopy(wpose))

            return waypoint

    def dual_plan_move(self, Group1_plan, Group2_plan):
        # UR3在前， UR10在后
        plan_whole_traj = moveit_msgs.msg.RobotTrajectory()
        plan_whole_traj.joint_trajectory.header.frame_id = 'base_step'
        plan_whole_traj.joint_trajectory.joint_names = ['ur10_shoulder_pan_joint', 'ur10_shoulder_lift_joint',
                                                        'ur10_elbow_joint',
                                                        'ur10_wrist_1_joint', 'ur10_wrist_2_joint',
                                                        'ur10_wrist_3_joint', 'ur3_shoulder_pan_joint',
                                                        'ur3_shoulder_lift_joint', 'ur3_elbow_joint',
                                                        'ur3_wrist_1_joint', 'ur3_wrist_2_joint', 'ur3_wrist_3_joint']
        traj_point = trajectory_msgs.msg.JointTrajectoryPoint()

        len_Group1_plan = len(Group1_plan.joint_trajectory.points)
        len_Group2_plan = len(Group2_plan.joint_trajectory.points)
        print len_Group1_plan
        print len_Group2_plan
        if len_Group1_plan > len_Group2_plan:
            for i in range(len(Group1_plan.joint_trajectory.points)):
                if i < len_Group2_plan:
                    traj_point.positions = numpy.concatenate((Group2_plan.joint_trajectory.points[i].positions,
                                                              Group1_plan.joint_trajectory.points[
                                                                  i].positions)).tolist()
                    traj_point.velocities = numpy.concatenate((Group2_plan.joint_trajectory.points[i].velocities,
                                                               Group1_plan.joint_trajectory.points[
                                                                   i].velocities)).tolist()
                    traj_point.accelerations = numpy.concatenate((Group2_plan.joint_trajectory.points[i].accelerations,
                                                                  Group1_plan.joint_trajectory.points[
                                                                      i].accelerations)).tolist()
                    traj_point.time_from_start = Group1_plan.joint_trajectory.points[i].time_from_start
                    plan_whole_traj.joint_trajectory.points.append(copy.deepcopy(traj_point))
                else:
                    traj_point.positions = numpy.concatenate((Group2_plan.joint_trajectory.points[
                                                                  len_Group2_plan - 1].positions,
                                                              Group1_plan.joint_trajectory.points[
                                                                  i].positions)).tolist()
                    traj_point.velocities = numpy.concatenate((Group2_plan.joint_trajectory.points[
                                                                   len_Group2_plan - 1].velocities,
                                                               Group1_plan.joint_trajectory.points[
                                                                   i].velocities)).tolist()
                    traj_point.accelerations = numpy.concatenate((Group2_plan.joint_trajectory.points[
                                                                      len_Group2_plan - 1].accelerations,
                                                                  Group1_plan.joint_trajectory.points[
                                                                      i].accelerations)).tolist()
                    traj_point.time_from_start = Group1_plan.joint_trajectory.points[i].time_from_start
                    plan_whole_traj.joint_trajectory.points.append(copy.deepcopy(traj_point))
        else:
            for i in range(len(Group2_plan.joint_trajectory.points)):
                if i < len_Group1_plan:
                    traj_point.positions = numpy.concatenate((Group2_plan.joint_trajectory.points[i].positions,
                                                              Group1_plan.joint_trajectory.points[
                                                                  i].positions)).tolist()
                    traj_point.velocities = numpy.concatenate((Group2_plan.joint_trajectory.points[i].velocities,
                                                               Group1_plan.joint_trajectory.points[
                                                                   i].velocities)).tolist()
                    traj_point.accelerations = numpy.concatenate((Group2_plan.joint_trajectory.points[i].accelerations,
                                                                  Group1_plan.joint_trajectory.points[
                                                                      i].accelerations)).tolist()
                    traj_point.time_from_start = Group2_plan.joint_trajectory.points[i].time_from_start
                    plan_whole_traj.joint_trajectory.points.append(copy.deepcopy(traj_point))
                else:
                    traj_point.positions = numpy.concatenate((Group2_plan.joint_trajectory.points[i].positions,
                                                              Group1_plan.joint_trajectory.points[
                                                                  len_Group1_plan - 1].positions)).tolist()
                    traj_point.velocities = numpy.concatenate((Group2_plan.joint_trajectory.points[i].velocities,
                                                               Group1_plan.joint_trajectory.points[
                                                                   len_Group1_plan - 1].velocities)).tolist()
                    traj_point.accelerations = numpy.concatenate((Group2_plan.joint_trajectory.points[i].accelerations,
                                                                  Group1_plan.joint_trajectory.points[
                                                                      len_Group1_plan - 1].accelerations)).tolist()
                    traj_point.time_from_start = Group2_plan.joint_trajectory.points[i].time_from_start
                    plan_whole_traj.joint_trajectory.points.append(copy.deepcopy(traj_point))

        return plan_whole_traj