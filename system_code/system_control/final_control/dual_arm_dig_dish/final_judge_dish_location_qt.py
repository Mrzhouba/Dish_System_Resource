#!/usr/bin/env python
#coding=UTF-8
import rospy
import sys, math, copy
from playsound import playsound
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/assist_modules')

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from final_trajectory_plan_qt import final_trajectory_plan_qt


from sensor_msgs.msg import JointState,Image,CameraInfo, PointCloud2
import open3d as o3d
import numpy as np
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import matplotlib.pyplot as plt
import numpy
from open3d_ros_helper import open3d_ros_helper as orh
import tf, time, json
import argparse
import Variable

UR10_Q1_to_left = Variable.UR10_Q1_to_left
# UR10_Q1 = [-0.16467219987978154, -1.430373493825094, 0.9728174209594727, -1.656503979359762, -1.4693554083453577, -0.9290836493121546]

# parser = argparse.ArgumentParser(description='UR10 dig dish paraamenters')
# parser.add_argument('--empty_depth', type=float, default=0.638144)
# parser.add_argument('--ur10_pull_dish_tra_path', type=str, default='/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/'
#                                                                  'saves_trajectory_mess/pull_dish_trajectory/ur10_high_trajectory.json')
# parser.add_argument('--camera_angele', type=float, default=0.507)
#
# dig_dish_args = parser.parse_args()
final_trajectory_plan_qt = final_trajectory_plan_qt()
volume_goal = Variable.require_volume
max_depth = Variable.empty_depth - Variable.real_max_depth
real_max_depth = Variable.real_max_depth

ur10_pull_to_initial_plan = Variable.ur10_pull_to_initial_plan

cut_safe_area_xbound_min = Variable.cut_safe_area_xbound_min
cut_safe_area_xbound_max = Variable.cut_safe_area_xbound_max
cut_safe_area_ybound_min = Variable.cut_safe_area_ybound_min
cut_safe_area_ybound_max = Variable.cut_safe_area_ybound_max
cut_safe_area_z = Variable.cut_safe_area_z

class final_judge_dish_location_qt():
    def __init__(self):
        self.camera_angele = Variable.camera_angele
        self.empty_depth = Variable.empty_depth
        self.ur10_pull_dish_tra_path = Variable.ur10_pull_dish_tra_path


        self.test = False

        if self.test == True:
            self.is_save_cloud = False
            self.is_dig = False
        else:
            self.is_save_cloud = False
            self.is_dig = True

        self.is_normalize = False
        self.monitor_push = []
        self.can_judge = True

        self.ur10_pull_dish_plan = self.read_tra_message(self.ur10_pull_dish_tra_path)
        self.ur10_shake_plan = self.read_tra_message(Variable.ur10_shake_plan)
        # self.ur10_pull_to_initial_plan = self.read_tra_message(ur10_pull_to_initial_plan)


    def points_callback(self, data, args, pre_need_push, current_dish_name):
        ur10_arm_spoon = args[0]
        ur10_spoon_end_effector_link = args[1]
        signal_ur10_send = args[2]
        signal_ur3_receive = args[3]
        get_model_output = args[4]
        signal_stop_receive = args[5]
        # add_dish_beep_send = args[6]
        joints_name = ur10_arm_spoon.get_joints()

        original_pointcloud = orh.rospc_to_o3dpc(data)
        rotate_pointcloud = self.pcd_rot(original_pointcloud, 0, self.camera_angele, 0)
        yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])

        test_pointcloud = self.cut_safe_area_pointcloud_2200(rotate_pointcloud)
        cal_volum_pointcloud = self.cut_safe_area_pointcloud_5000(rotate_pointcloud)

        cloud_point_data = np.asarray(test_pointcloud.points)
        x_boundary = (np.min(cloud_point_data[:, 0]), np.max(cloud_point_data[:, 0]))
        y_boundary = (np.min(cloud_point_data[:, 1]), np.max(cloud_point_data[:, 1]))

        check_push_pointcloud = test_pointcloud.voxel_down_sample(voxel_size=0.02)

        if pre_need_push == True:
            push_index = self.judge_need_push(check_push_pointcloud, x_boundary, y_boundary, None)
        else:
            # push_index = None
            if self.can_judge == True:
                push_index = self.judge_need_push(check_push_pointcloud, x_boundary, y_boundary, 0.025)
            else:
                push_index = None

        if push_index != None:
            print 'initial push'
            action_name = 'initial push'
            waypoints = push_index
            orientation_waypoints = None
            self.monitor_push.append(waypoints)

            self.can_judge = False

            if len(self.monitor_push) != 1:
                if waypoints == self.monitor_push[0] and len(self.monitor_push) == 2:
                    print '俩次相同位置，需要补菜'
                    del self.monitor_push[:]
                    # add_dish_beep_send.send('need_add_dish')

                    signal_stop_receive.send('add_stop')

                    rospy.sleep(1)
                    return False, False

                elif len(self.monitor_push) > 2:
                    print '推3次，需要补菜'
                    del self.monitor_push[:]
                    # add_dish_beep_send.send('need_add_dish')

                    signal_stop_receive.send('add_stop')

                    rospy.sleep(1)
                    return False, False

            if self.is_dig == True:
                is_completed = final_trajectory_plan_qt.push_action_trajectory_plan(action_name, waypoints,
                                                                                    orientation_waypoints,
                                                                                    ur10_arm_spoon,
                                                                                    ur10_spoon_end_effector_link)
                return is_completed, False
        else:
            points = np.asarray(test_pointcloud.points, dtype=float)
            points_color = np.asarray(test_pointcloud.colors, dtype=float)
            choice = np.random.choice(len(points), 2500, replace=True)
            xyz_points = points[choice, :]
            xyz_points_color = points_color[choice, :]

            cal_volum_points = np.asarray(cal_volum_pointcloud.points, dtype=float)
            cal_volum_points_color = np.asarray(cal_volum_pointcloud.colors, dtype=float)

            input_model_data = points

            npoints_pointcloud = o3d.geometry.PointCloud()
            npoints_pointcloud.points = o3d.utility.Vector3dVector(points)
            npoints_pointcloud.colors = o3d.utility.Vector3dVector(points_color)

            cal_volum_npoints_pointcloud = o3d.geometry.PointCloud()
            cal_volum_npoints_pointcloud.points = o3d.utility.Vector3dVector(cal_volum_points)
            cal_volum_npoints_pointcloud.colors = o3d.utility.Vector3dVector(cal_volum_points_color)
            # npoints_pointcloud.paint_uniform_color([0.5,0.5,0.5])
            # o3d.visualization.draw_geometries([npoints_pointcloud, yuandian])

            # 得到模型输出
            model_result = get_model_output.get_result(input_model_data)

            coloring_pointcloud, dig_area_pointcloud, analyze_start_location, analyze_end_location, dig_move_pass_points, action_name, \
                volume, sound_index, real_ave_depth = self.analyze_model_reuslt(npoints_pointcloud, model_result,
                                                                                cal_volum_npoints_pointcloud, x_boundary,
                                                                                current_dish_name)

            if dig_area_pointcloud is not None and action_name != 'corner push' and volume > 1500 and sound_index != 1:
                del self.monitor_push[:]
                self.can_judge = True
                start_location, end_location = self.check_point_safely(x_boundary, y_boundary, analyze_start_location,
                                                                       analyze_end_location)

                # print start_location, end_location
                # volume = self.compute_cloud_volume(dig_area_pointcloud)
                print "dig area volum:" + str(volume)

                # 存储点云
                if self.is_save_cloud == True:
                    self.save_pointcloud(rotate_pointcloud)

                real_coloring_pointcloud = self.pcd_rot(coloring_pointcloud, 0, -self.camera_angele, 0)
                real_dig_area_pointcloud = self.pcd_rot(dig_area_pointcloud, 0, -self.camera_angele, 0)
                real_start_location = self.rotate_by_axis_y(start_location, -self.camera_angele)
                real_end_location = self.rotate_by_axis_y(end_location, -self.camera_angele)


                dig_type = self.judge_dig_action(dig_area_pointcloud, 0.04, 0.7)

                if action_name == 'dig dish':
                    print action_name
                    print dig_type
                    waypoints, orientation_waypoints = self.return_waypoints(dig_type, start_location, end_location,
                                                                             dig_move_pass_points, volume, cal_volum_pointcloud, x_boundary,
                                                                             dig_area_pointcloud, current_dish_name)
                    # print waypoints[-1]
                    # print orientation_waypoints

                    if self.is_dig == True:
                        is_completed, move_dish_plan, is_need_push = final_trajectory_plan_qt.push_action_trajectory_plan(
                            action_name, waypoints, orientation_waypoints, ur10_arm_spoon, ur10_spoon_end_effector_link)

                        if is_completed == True and move_dish_plan != None:
                            # 同时到达
                            drop_bowl_finish = signal_ur3_receive.recv()

                            signal_ur10_send.send("get to pull loc")
                            ur10_arm_spoon.execute(move_dish_plan)

                            if signal_ur3_receive.recv() == "bowl is ready":
                                signal_ur10_send.send("start pull dish")

                                ur10_arm_spoon.execute(self.ur10_pull_dish_plan)
                                # rospy.sleep(1)

                                ur10_arm_spoon.set_joint_value_target(Variable.ur10_pull_finish)
                                ur10_arm_spoon.go()

                                is_need_shake = True
                                if(current_dish_name == "JiMiHua"):
                                    is_need_shake = False

                                if is_need_shake == True:
                                    ur10_arm_spoon.execute(self.ur10_shake_plan)
                                    rospy.sleep(0.1)

                                ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
                                ur10_pull_to_start_plan = ur10_arm_spoon.plan()

                                if len(ur10_pull_to_start_plan.joint_trajectory.points) == 0:
                                    print "读轨迹回初始位置"
                                    ur10_arm_spoon.set_joint_value_target(Variable.ur10_doudong1)
                                    ur10_arm_spoon.go()
                                    ur10_pull_to_start_plan = ur10_pull_to_initial_plan

                                signal_ur10_send.send("shake finish")

                                ur10_arm_spoon.execute(ur10_pull_to_start_plan)

                                # ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
                                # ur10_arm_spoon.go()

                                return True, is_need_push

                        else:
                            ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
                            ur10_arm_spoon.go()
                            return False, is_need_push

                    if self.test == True:
                        real_start_x, real_start_y = self.get_rotate_angle_to_end(orientation_waypoints[0], orientation_waypoints[1])
                        real_middle_x, real_middle_y = self.get_rotate_angle_to_end(orientation_waypoints[1],
                                                                                  orientation_waypoints[2])
                        real_end_x, real_end_y = self.get_rotate_angle_to_end(orientation_waypoints[2],
                                                                                  orientation_waypoints[3])

                        start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[0])
                        middle_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[1])
                        end_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[2])

                        theta_z = math.atan2(real_end_location[0] - real_start_location[0], real_end_location[1] - real_start_location[1])

                        p = start_coor.get_rotation_matrix_from_xyz((real_start_x, real_start_y, 0))
                        r = start_coor.get_rotation_matrix_from_xyz((real_middle_x, real_middle_y, 0))
                        q = start_coor.get_rotation_matrix_from_xyz((real_end_x, real_end_y, math.pi / 2 + theta_z))

                        start_coor.rotate(p)
                        middle_coor.rotate(r)
                        end_coor.rotate(q)

                        o3d.visualization.draw_geometries([real_dig_area_pointcloud, real_coloring_pointcloud, yuandian, start_coor, middle_coor, end_coor])
                        return False, False

            elif sound_index == 1:
                del self.monitor_push[:]
                # add_dish_beep_send.send('need_add_dish')

                signal_stop_receive.send('add_stop')

                rospy.sleep(1)
                return False, False

            else:
                waypoints = self.judge_need_push(check_push_pointcloud, x_boundary, y_boundary, None)

                self.monitor_push.append(waypoints)
                self.can_judge = False

                if len(self.monitor_push) != 1:
                    if waypoints == self.monitor_push[0] and len(self.monitor_push) == 2:
                        print '俩次相同位置，需要补菜'
                        del self.monitor_push[:]
                        # add_dish_beep_send.send('need_add_dish')

                        signal_stop_receive.send('add_stop')

                        rospy.sleep(1)
                        return False, False

                    elif len(self.monitor_push) > 2:
                        print '推3次，需要补菜'
                        del self.monitor_push[:]
                        # add_dish_beep_send.send('need_add_dish')

                        signal_stop_receive.send('add_stop')

                        rospy.sleep(1)
                        return False, False

                action_name = 'initial push'
                orientation_waypoints = None
                if self.is_dig == True:
                    if waypoints != None:
                        is_completed = final_trajectory_plan_qt.push_action_trajectory_plan(action_name, waypoints, orientation_waypoints,
                                                                                                    ur10_arm_spoon,
                                                                                                    ur10_spoon_end_effector_link)
                        return is_completed, False
                    else:
                        return False, False



    def judge_need_push(self, cloud, x_boundary, y_boundary, tolerance):
        total = 0

        for i in range(len(cloud.points)):
            total = total + self.empty_depth - cloud.points[i][2]
        aver_total = total * 1.0 / len(cloud.points)

        print '全体平均深度', aver_total
        if aver_total > 0.048 and tolerance != None:
            max_index = None
        else:
            x_three = []
            y_three = []
            for i in range(4):
                x_three.append(i * (x_boundary[1] - x_boundary[0]) / 3 + x_boundary[0])
                y_three.append(i * (y_boundary[1] - y_boundary[0]) / 3 + y_boundary[0])

            center_points_depth = 0
            center_num = 0
            four_corner_points_depth = [0, 0, 0, 0]
            four_corner_num = [0, 0, 0, 0]
            for i in range(len(cloud.points)):
                if cloud.points[i][0] > x_three[2] and cloud.points[i][1] < y_three[1]:
                    four_corner_points_depth[0] = four_corner_points_depth[0] + (
                            self.empty_depth - cloud.points[i][2])
                    four_corner_num[0] = four_corner_num[0] + 1

                elif cloud.points[i][0] > x_three[2] and cloud.points[i][1] > y_three[2]:
                    four_corner_points_depth[1] = four_corner_points_depth[1] + (
                            self.empty_depth - cloud.points[i][2])
                    four_corner_num[1] = four_corner_num[1] + 1

                elif cloud.points[i][0] < x_three[1] and cloud.points[i][1] > y_three[2]:
                    four_corner_points_depth[2] = four_corner_points_depth[2] + (
                            self.empty_depth - cloud.points[i][2])
                    four_corner_num[2] = four_corner_num[2] + 1

                elif cloud.points[i][0] < x_three[1] and cloud.points[i][1] < y_three[1]:
                    four_corner_points_depth[3] = four_corner_points_depth[3] + (
                            self.empty_depth - cloud.points[i][2])
                    four_corner_num[3] = four_corner_num[3] + 1

                elif x_three[1] < cloud.points[i][0] < x_three[2] and y_three[1] < cloud.points[i][1] < y_three[2]:
                    center_points_depth = center_points_depth + (self.empty_depth - cloud.points[i][2])
                    center_num = center_num + 1

            center_ave_depth = center_points_depth * 1.0 / center_num

            print 'center_ave_depth', center_ave_depth

            satisfied_target_area = []
            satisfied_target_area_index = []
            save_index2_value = 0
            for i in range(4):
                aver_depth = four_corner_points_depth[i] * 1.0 / four_corner_num[i]

                print '角落深度', aver_depth
                if i == 2:
                    save_index2_value = aver_depth
                # if i == 0 and aver_depth > 0.04:
                #     aver_depth = aver_depth + 0.01
                # if i == 3 and aver_depth > 0.035:
                #     aver_depth = aver_depth - 0.01
                if tolerance != None:
                    if aver_depth > 0.04:
                        if i == 0 or i == 1 or i == 3:
                            aver_depth = aver_depth + 0.01
                        elif i == 2:
                            aver_depth = aver_depth - 0.015
                else:
                    if i == 0 or i == 1 or i == 3:
                        aver_depth = aver_depth + 0.01
                    elif i == 2:
                        aver_depth = aver_depth - 0.015

                if tolerance == None:
                    satisfied_target_area.append(aver_depth)
                else:
                    if aver_depth - center_ave_depth > tolerance:
                        satisfied_target_area.append(aver_depth)
                        satisfied_target_area_index.append(i)
                    else:
                        satisfied_target_area.append(aver_depth)

            # for i in range(len(satisfied_target_area)):
            #     if satisfied_target_area[i] > 0.04 and i != 2:
            #         satisfied_target_area[2] = 0

            if tolerance == None:
                max_index = satisfied_target_area.index(max(satisfied_target_area))
            else:
                if len(satisfied_target_area_index) != 0:
                    max_index = satisfied_target_area.index(max(satisfied_target_area))
                else:
                    max_index = None

            if tolerance == None:
                if max_index == 2:
                    print '识别失败推第二高'
                    satisfied_target_area[2] = 0
                    max_index = satisfied_target_area.index(max(satisfied_target_area))
            else:
                if max_index == 2:
                    if save_index2_value < 0.065:
                        print '放弃推右下角'
                        satisfied_target_area[2] = 0
                        max_index = satisfied_target_area.index(max(satisfied_target_area))
                        if satisfied_target_area[max_index] < 0.025:
                            print '放弃推第二高角落'
                            max_index = None

        return max_index

    def return_push_waypoints(self, start_point, end_point, dig_move_pass_points):
        waypoints = []
        orientation_waypoints = []
        np_move_pass_points = np.asarray(dig_move_pass_points)
        if end_point[1] - start_point[1] < 0:
            order_move_pass_points = np_move_pass_points[np_move_pass_points[:, 1].argsort()]
        else:
            order_move_pass_points = np_move_pass_points[np.argsort(-np_move_pass_points[:, 1]), :]

        # print order_move_pass_points
        num = len(order_move_pass_points)

        push_start_point = [end_point[0], end_point[1], end_point[2]-0.05]
        real_push_start_location = self.rotate_by_axis_y(push_start_point, -self.camera_angele)
        waypoints.append(real_push_start_location)
        real_start_push_point = self.rotate_by_axis_y(end_point, -self.camera_angele)
        orientation_waypoints.append(real_start_push_point)


        push_middle_point = order_move_pass_points[int(num/5)]
        deal_push_middle_point = [push_middle_point[0], push_middle_point[1], self.empty_depth-0.06]
        real_push_middle_location = self.rotate_by_axis_y(deal_push_middle_point, -self.camera_angele)
        waypoints.append(real_push_middle_location)

        push_end_point = [start_point[0], start_point[1], self.empty_depth - 0.05]
        real_push_end_location = self.rotate_by_axis_y(push_end_point, -self.camera_angele)
        waypoints.append(real_push_end_location)
        real_end_push_point = self.rotate_by_axis_y(start_point, -self.camera_angele)
        orientation_waypoints.append([real_end_push_point[0], real_end_push_point[1], real_start_push_point[2]])

        return waypoints, orientation_waypoints

    def return_waypoints(self, dig_type, start_point, end_point, move_pass_points, volum, cal_volum_pointcloud, x_boundary, dig_area_pointcloud, current_dish_name):
        np_move_pass_points = np.asarray(move_pass_points)
        if end_point[1] - start_point[1] > 0:
            order_move_pass_points = np_move_pass_points[np_move_pass_points[:,1].argsort()]
        else:
            order_move_pass_points = np_move_pass_points[np.argsort(-np_move_pass_points[:,1]), :]

        waypoints = []
        orientation_waypoints = []
        num = len(order_move_pass_points)

        if dig_type == 'full rotate':
            base_height = (start_point[2]-real_max_depth)-(1-(self.empty_depth-start_point[2])/7)*0.02
            print 'start point z', base_height

            deal_start_point = [start_point[0], start_point[1], base_height]
            # deal_start_point = [start_point[0], start_point[1], start_point[2] - 0.055]
            real_start_location = self.rotate_by_axis_y(deal_start_point, -self.camera_angele)
            no_deal_real_start_location = self.rotate_by_axis_y(start_point, -self.camera_angele)
            waypoints.append(real_start_location)
            orientation_waypoints.append(no_deal_real_start_location)

            middle_point = order_move_pass_points[int(num/2)]
            #0.599 0.6 0.535
            #0.529 0.61 0.55

            if middle_point[2] > max_depth:
                deal_middle_point = [middle_point[0], middle_point[1], max_depth]
            else:
                deal_middle_point = [middle_point[0], middle_point[1], middle_point[2]]

            if middle_point[2] > max_depth:
                deal_middle_point = [middle_point[0], middle_point[1], max_depth]

            print '中间点深度', deal_middle_point
            real_middle_location = self.rotate_by_axis_y(deal_middle_point, -self.camera_angele)
            waypoints.append(real_middle_location)

            orientation_middle_location = self.rotate_by_axis_y((middle_point[0], middle_point[1], start_point[2]+0.01), -self.camera_angele)
            orientation_waypoints.append(orientation_middle_location)

            end_point_z = self.end_point_z_judge(start_point, end_point, cal_volum_pointcloud, volum, x_boundary, dig_area_pointcloud, current_dish_name)
            deal_end_point = [end_point[0], end_point[1], end_point_z]
            real_end_location = self.rotate_by_axis_y(deal_end_point, -self.camera_angele)
            # no_deal_real_end_location = self.rotate_by_axis_y(deal_end_point, -self.camera_angele)
            waypoints.append(real_end_location)

            orientation_end_location = self.rotate_by_axis_y((end_point[0], end_point[1], start_point[2]), -self.camera_angele)
            orientation_last_location = self.rotate_by_axis_y((end_point[0], end_point[1], 0.5), -self.camera_angele)
            orientation_waypoints.append(orientation_end_location)
            orientation_waypoints.append(orientation_last_location)


        elif dig_type == 'push rotate':
            base_height = (start_point[2]-real_max_depth)-(1-(self.empty_depth-start_point[2])/7)*0.02
            print 'start point z', base_height

            deal_start_point = [start_point[0], start_point[1], base_height]
            # deal_start_point = [start_point[0], start_point[1], start_point[2] - 0.055]
            real_start_location = self.rotate_by_axis_y(deal_start_point, -self.camera_angele)
            no_deal_real_start_location = self.rotate_by_axis_y(start_point, -self.camera_angele)
            waypoints.append(real_start_location)
            orientation_waypoints.append(no_deal_real_start_location)

            middle_point = order_move_pass_points[int(num / 2)]
            # 0.599 0.6 0.535
            # 0.529 0.61 0.55
            print middle_point[2]
            if middle_point[2] > max_depth:
                deal_middle_point = [middle_point[0], middle_point[1], max_depth]
            else:
                deal_middle_point = [middle_point[0], middle_point[1], middle_point[2]]

            if middle_point[2] > max_depth:
                deal_middle_point = [middle_point[0], middle_point[1], max_depth]
            print '中间点深度', deal_middle_point

            real_middle_location = self.rotate_by_axis_y(deal_middle_point, -self.camera_angele)
            waypoints.append(real_middle_location)

            orientation_middle_location = self.rotate_by_axis_y((middle_point[0], middle_point[1], start_point[2]+0.01),
                                                                -self.camera_angele)
            orientation_waypoints.append(orientation_middle_location)

            end_point_z = self.end_point_z_judge(start_point, end_point, cal_volum_pointcloud, volum, x_boundary, dig_area_pointcloud, current_dish_name)
            deal_end_point = [end_point[0], end_point[1], end_point_z]
            real_end_location = self.rotate_by_axis_y(deal_end_point, -self.camera_angele)
            # no_deal_real_end_location = self.rotate_by_axis_y(deal_end_point, -self.camera_angele)
            waypoints.append(real_end_location)

            orientation_end_location = self.rotate_by_axis_y((end_point[0], end_point[1], start_point[2]), -self.camera_angele)
            orientation_last_location = self.rotate_by_axis_y((end_point[0], end_point[1], 0.5), -self.camera_angele)
            orientation_waypoints.append(orientation_end_location)
            orientation_waypoints.append(orientation_last_location)


        return waypoints, orientation_waypoints

    def end_point_z_judge(self, start_point, end_point, cal_volum_pointcloud, volum, x_boundary, dig_area_pointcloud, current_dish_name):
        square_points = []
        square_pointcloud = o3d.geometry.PointCloud()
        len_points = len(cal_volum_pointcloud.points)

        for i in range(len_points):
            if abs(cal_volum_pointcloud.points[i][0] - end_point[0]) <= 0.035 and abs(cal_volum_pointcloud.points[i][1] - end_point[1]) <= 0.035:
                square_points.append(cal_volum_pointcloud.points[i])

        square_pointcloud.points = o3d.utility.Vector3dVector(square_points)
        # o3d.visualization.draw_geometries([square_pointcloud])

        total_depth = 0
        for i in range(len(square_pointcloud.points)):
            total_depth = (self.empty_depth - square_pointcloud.points[i][2]) + total_depth

        ave_depth = total_depth / len(square_pointcloud.points)
        print '终点平均深度', ave_depth

        dig_total_depth = 0
        for i in range(len(dig_area_pointcloud.points)):
            dig_total_depth = (self.empty_depth - dig_area_pointcloud.points[i][2]) + dig_total_depth

        dig_ave_depth = dig_total_depth / len(dig_area_pointcloud.points)
        print '打菜区域平均深度', dig_ave_depth

        allowMax_height = end_point[2] - real_max_depth
        if dig_ave_depth > ave_depth or (ave_depth > 0.045 and dig_ave_depth > 0.035):
            if current_dish_name == 'ChaoPuGua' or current_dish_name == 'RouPiHuangDou' or current_dish_name == 'ZhaTuDou':
                end_point_z = allowMax_height
                print 'allowmax'
            else:
                print 'allowmax+0.01'
                end_point_z = allowMax_height + 0.01
        else:
            print '公式'
            end_point_z = (max_depth - allowMax_height) * (ave_depth / dig_ave_depth - ave_depth/0.035) + allowMax_height

        if ave_depth > 0.05 and dig_ave_depth < 0.025:
            print 'end_point[2]'
            end_point_z = end_point[2]

        if end_point_z > max_depth:
            print 'maxdepth'
            end_point_z = max_depth
        print 'end_point_z', end_point_z
        return end_point_z

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

    def compute_cloud_volume(self, cloud):
        # real_cloud = self.pcd_rot(cloud, 0, -camera_angele, 0)
        density = 31 * 46 * 1.0 / 2500
        real_depth = 0
        for i in range(len(cloud.points)):
            depth = (-cloud.points[i][2] + self.empty_depth) * 100
            real_depth = real_depth + depth
        volume = real_depth * density
        return volume


    def check_point_safely(self, x_boundary, y_boundary, analyze_start_location, analyze_end_location):
        start_location = list(analyze_start_location)
        end_location = list(analyze_end_location)

        if start_location[0] - x_boundary[0] < real_max_depth:
            print "start x outrange"
            start_location[0] = x_boundary[0] + real_max_depth
        if x_boundary[1] - start_location[0] < real_max_depth:
            print "start x outrange"
            start_location[0] = x_boundary[1] - real_max_depth

        if end_location[0] - x_boundary[0] < real_max_depth:
            print "end x outrange"
            end_location[0] = x_boundary[0] + real_max_depth
        if x_boundary[1] - end_location[0] < real_max_depth:
            print "end x outrange"
            end_location[0] = x_boundary[1] - real_max_depth

        if start_location[1] - y_boundary[0] < real_max_depth:
            print "start y outrange"
            start_location[1] = y_boundary[0] + real_max_depth
        if y_boundary[1] - start_location[1] < real_max_depth:
            print "start y outrange"
            start_location[1] = y_boundary[1] - real_max_depth

        if end_location[1] - y_boundary[0] < real_max_depth:
            print "end y outrange"
            end_location[1] = y_boundary[0] + real_max_depth
        if y_boundary[1] - end_location[1] < real_max_depth:
            print "end y outrange"
            end_location[1] = y_boundary[1] - real_max_depth
        return start_location, end_location



    def compute_rotate_matrix(self, angle_x, angle_y, angle_z, rotate_order):
        matrix_z = [[math.cos(angle_z), -math.sin(angle_z), 0, 0],
                    [math.sin(angle_z), math.cos(angle_z), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        matrix_x = [[1, 0, 0, 0], [0, math.cos(angle_x), -math.sin(angle_x), 0],
                    [0, math.sin(angle_x), math.cos(angle_x), 0], [0, 0, 0, 1]]
        matrix_y = [[math.cos(angle_y), 0, math.sin(angle_y), 0], [0, 1, 0, 0],
                    [-math.sin(angle_y), 0, math.cos(angle_y), 0], [0, 0, 0, 1]]

        if rotate_order == 'xyz':
            matrix_temp = numpy.dot(matrix_x, matrix_y)
            matrix = numpy.dot(matrix_temp, matrix_z)
            input_quaternion = tf.transformations.quaternion_from_matrix(matrix)

        elif rotate_order == 'xzy':
            matrix_temp = numpy.dot(matrix_x, matrix_z)
            matrix = numpy.dot(matrix_temp, matrix_y)
            input_quaternion = tf.transformations.quaternion_from_matrix(matrix)

        elif rotate_order == 'yxz':
            matrix_temp = numpy.dot(matrix_y, matrix_x)
            matrix = numpy.dot(matrix_temp, matrix_z)
            input_quaternion = tf.transformations.quaternion_from_matrix(matrix)

        elif rotate_order == 'yzx':
            matrix_temp = numpy.dot(matrix_y, matrix_z)
            matrix = numpy.dot(matrix_temp, matrix_x)
            input_quaternion = tf.transformations.quaternion_from_matrix(matrix)

        elif rotate_order == 'zxy':
            matrix_temp = numpy.dot(matrix_z, matrix_x)
            matrix = numpy.dot(matrix_temp, matrix_y)
            input_quaternion = tf.transformations.quaternion_from_matrix(matrix)

        elif rotate_order == 'zyx':
            matrix_temp = numpy.dot(matrix_z, matrix_y)
            matrix = numpy.dot(matrix_temp, matrix_x)
            input_quaternion = tf.transformations.quaternion_from_matrix(matrix)

        return input_quaternion

    def save_pointcloud(self, cloud):
        path = '/home/robot/sse-images/dig_data_rgb/'
        current_time = time.strftime('%Y%m%d%H%M%S', time.localtime(time.time()))
        deal_cloud = self.cut_remain_rgb_pointcloud(cloud)
        # o3d.visualization.draw_geometries([deal_cloud])
        o3d.io.write_point_cloud(path + 'process_pointcloud_' + current_time + '.pcd', deal_cloud)


    def cut_remain_rgb_pointcloud(self, cloud):
        points = np.asarray(cloud.points)
        dish_area_pointcloud = o3d.geometry.PointCloud()
        still_points = []
        still_points_rgb = []
        for i in range(len(points)):
            if (0.11  < points[i][0] < 0.55  and -0.185 < points[i][1] < 0.11 and points[i][
                2] > 0.5):
                still_points.append(points[i])
                still_points_rgb.append(cloud.colors[i])

        dish_area_pointcloud.points = o3d.utility.Vector3dVector(still_points)
        dish_area_pointcloud.colors = o3d.utility.Vector3dVector(still_points_rgb)
        # dish_area_pointcloud = dish_area_pointcloud.voxel_down_sample(voxel_size=0.005)
        # print len(dish_area_pointcloud.points)
        return dish_area_pointcloud

    def get_rotate_angle_to_end(self, start_loc, end_loc):
        delta_x = end_loc[0] - start_loc[0]
        delta_y = end_loc[1] - start_loc[1]
        delta_z = end_loc[2] - start_loc[2]
        # print delta_z
        angle_x = math.atan2(delta_y, delta_z)
        y_z = math.sqrt(pow(delta_z, 2) + pow(delta_y, 2))
        angle_y = math.atan2(delta_x, y_z)

        return -angle_x, angle_y



    def judge_dig_action(self, middle_area_pointcloud, bottom_threshold, proportion):
        num = len(middle_area_pointcloud.points)
        satisfy_bottom_dis_num = 0
        for i in range(num):

            if (self.empty_depth-middle_area_pointcloud.points[i][2] > bottom_threshold):
                satisfy_bottom_dis_num += 1
        if satisfy_bottom_dis_num*1.0/num > proportion:
            return 'full rotate'
        else:
            return 'push rotate'

    def cut_safe_area_pointcloud_2200(self, cloud):
        points = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors)
        dish_area_pointcloud = o3d.geometry.PointCloud()
        still_points = []
        still_colors = []
        for i in range(len(points)):
            # if (0.081 < points[i][0] < 0.57 and -0.2 < points[i][1] < 0.15 and points[i][2] > 0.5):
            if (cut_safe_area_xbound_min < points[i][0] < cut_safe_area_xbound_max
                    and cut_safe_area_ybound_min < points[i][1] < cut_safe_area_ybound_max and points[i][2] > cut_safe_area_z):
                still_points.append(points[i])
                still_colors.append(colors[i])

        dish_area_pointcloud.points = o3d.utility.Vector3dVector(still_points)
        dish_area_pointcloud.colors = o3d.utility.Vector3dVector(still_colors)
        # o3d.visualization.draw_geometries([dish_area_pointcloud])

        dish_area_pointcloud = dish_area_pointcloud.voxel_down_sample(voxel_size=0.0085)
        print 'test pointcloud下采样后点数', len(dish_area_pointcloud.points)
        # rospy.sleep(11111111111111)
        return dish_area_pointcloud

    def cut_safe_area_pointcloud_5000(self, cloud):
        points = np.asarray(cloud.points)
        colors = np.asarray(cloud.colors)
        dish_area_pointcloud = o3d.geometry.PointCloud()
        still_points = []
        still_colors = []
        for i in range(len(points)):
            # if (0.081 < points[i][0] < 0.57 and -0.2 < points[i][1] < 0.15 and points[i][2] > 0.5):
            if (cut_safe_area_xbound_min + 0.05 < points[i][0] < cut_safe_area_xbound_max-0.05
                    and cut_safe_area_ybound_min + 0.05 < points[i][1] < cut_safe_area_ybound_max - 0.05 and points[i][2] > cut_safe_area_z):
                still_points.append(points[i])
                still_colors.append(colors[i])

        dish_area_pointcloud.points = o3d.utility.Vector3dVector(still_points)
        dish_area_pointcloud.colors = o3d.utility.Vector3dVector(still_colors)
        dish_area_pointcloud = dish_area_pointcloud.voxel_down_sample(voxel_size=0.004)
        print 'cal volum pointcloud下采样后点数', len(dish_area_pointcloud.points)
        return dish_area_pointcloud

    def pcd_rot(self, pcd, theta_x, theta_y, theta_z):
        pcd_copy = copy.deepcopy(pcd)
        pcd_rot_x = pcd_copy.transform(
            [[1, 0, 0, 0], [0, math.cos(theta_x), -math.sin(theta_x), 0], [0, math.sin(theta_x), math.cos(theta_x), 0],
             [0, 0, 0, 1]])
        pcd_rot_x_y = pcd_rot_x.transform(
            [[math.cos(theta_y), 0, math.sin(theta_y), 0], [0, 1, 0, 0], [-math.sin(theta_y), 0, math.cos(theta_y), 0],
             [0, 0, 0, 1]])
        pcd_rot_x_y_z = pcd_rot_x_y.transform(
            [[math.cos(theta_z), -math.sin(theta_z), 0, 0], [math.sin(theta_z), math.cos(theta_z), 0, 0], [0, 0, 1, 0],
             [0, 0, 0, 1]])
        return pcd_rot_x_y_z


    def analyze_model_reuslt(self, cloud, model_result, cal_volum_npoints_pointcloud, x_boundary, current_dish_name):
        origin_color = np.asarray(cloud.colors)
        start_pointcloud = o3d.geometry.PointCloud()
        end_pointcloud = o3d.geometry.PointCloud()
        start_points = []
        end_points = []
        point_num = len(model_result)
        for i in range(point_num):
            if model_result[i] == 1 or model_result[i] == 4 or model_result[i] == 7 or model_result[i] == 10 \
                    or model_result[i] == 13 or model_result[i] == 16 or model_result[i] == 19:
                start_points.append(cloud.points[i])
                cloud.colors[i] = [1, 0, 0]
            elif model_result[i] == 2 or model_result[i] == 5 or model_result[i] == 8 or model_result[i] == 11 \
                    or model_result[i] == 14 or model_result[i] == 17 or model_result[i] == 20:
                end_points.append(cloud.points[i])
                cloud.colors[i] = [0, 1, 0]
            else:
                cloud.colors[i] = origin_color[i]
        start_pointcloud.points = o3d.utility.Vector3dVector(start_points)
        end_pointcloud.points = o3d.utility.Vector3dVector(end_points)

        print 'start_pointcloud点数', len(start_pointcloud.points)
        print 'end_pointcloud点数', len(end_pointcloud.points)

        start_center_point = start_pointcloud.get_center()
        end_center_point = end_pointcloud.get_center()
        print 'start centet point', start_center_point
        print 'end center point', end_center_point

        if start_center_point[1] - end_center_point[1] < -0.05 and start_center_point[0] - end_center_point[0] > 0.01:
            action_name = 'dig dish'
            require_volum = volume_goal[current_dish_name]
        elif start_center_point[1] - end_center_point[1] > 0.01 and start_center_point[0] - end_center_point[0] > 0.01:
            action_name = 'corner push'
            require_volum = 0
        elif abs(start_center_point[0] - end_center_point[0]) < 0.01:
            action_name = 'corner push'
            require_volum = 0
        else:
            action_name = 'corner push'
            require_volum = 0

        if len(start_pointcloud.points) > 0 and len(end_pointcloud.points) > 50 and action_name == 'dig dish':
            middle_area_pointcloud, dig_move_pass_points, start_location, end_location, volume, real_ave_depth = self.get_required_volum_area(start_center_point,
                                                                                                              end_center_point,
                                                                                                              0.1,
                                                                                                              require_volum,
                                                                                                              cal_volum_npoints_pointcloud,
                                                                                                              x_boundary)

            # middle_area_pointcloud_tree = o3d.geometry.KDTreeFlann(middle_area_pointcloud)
            # [_, start_idx, _] = middle_area_pointcloud_tree.search_knn_vector_3d(end_location, int(
            #     len(middle_area_pointcloud.points) / 5))
            # end_pointset = np.asarray(middle_area_pointcloud.points)[start_idx[1:], :]
            # end_location = [end_location[0], end_location[1], np.mean(end_pointset, 0)[2]]
            if volume > 1500:
                return cloud, middle_area_pointcloud, start_location, end_location, dig_move_pass_points, action_name, volume, 0, real_ave_depth
            elif volume < 1500 and real_ave_depth < 0.015:
                print '体积不到1500,且深度小于1.5cm,sound_index=1'
                return None, None, None, None, None, 'corner push', None, 1, real_ave_depth
            else:
                print '体积不到1500,深度大于1.5cm,push'
                return None, None, None, None, None, 'corner push', None, 0, real_ave_depth
        else:
            print '识别失败push,sound_index=0'
            return None, None, None, None, None, 'corner push', None, 0, None


    def get_required_volum_area(self, point_1, point_2, area_width, volum, cal_volum_npoints_pointcloud, x_boundary):
        yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
        middle_rotate_area = o3d.geometry.PointCloud()
        up_half_final_middle_rotate_area = o3d.geometry.PointCloud()
        final_middle_rotate_area = o3d.geometry.PointCloud()

        middle_rotate_area_points = []
        theta = math.atan2((point_1[1]-point_2[1]), (point_1[0]-point_2[0]))
        # o3d.visualization.draw_geometries([cloud, yuandian])

        total_depth = 0
        for i in range(len(cal_volum_npoints_pointcloud.points)):
            total_depth = cal_volum_npoints_pointcloud.points[i][2] + total_depth

        ave_depth = total_depth / len(cal_volum_npoints_pointcloud.points)
        real_ave_depth = self.empty_depth - ave_depth

        align_cal_volum_cloud = self.pcd_rot(cal_volum_npoints_pointcloud, 0, 0, -theta)
        # o3d.visualization.draw_geometries([align_cal_volum_cloud, yuandian])

        align_point_1 = self.rotate_by_axis_z(point_1, -theta)  #start
        align_point_2 = self.rotate_by_axis_z(point_2, -theta)
        # print align_point_1, align_point_2
        all_dig_move_pass_points = []

        for i in range(len(align_cal_volum_cloud.points)):
            if align_point_1[1] - 0.005 < align_cal_volum_cloud.points[i][1] < align_point_1[1] + 0.005:
                all_dig_move_pass_points.append(align_cal_volum_cloud.points[i])

            if align_point_2[1]-area_width/2 < align_cal_volum_cloud.points[i][1] < align_point_2[1]+area_width/2:
                # align_cloud.colors[i] = [0.5,0,0.1]
                middle_rotate_area_points.append(align_cal_volum_cloud.points[i])

        np_dig_move_pass_points = np.asarray(all_dig_move_pass_points)
        order_move_pass_points = np_dig_move_pass_points[np_dig_move_pass_points[:, 0].argsort()]
        up_x = order_move_pass_points[-1][0]
        bottom_x = order_move_pass_points[0][0]

        middle_rotate_area.points = o3d.utility.Vector3dVector(middle_rotate_area_points)
        middle_rotate_area.paint_uniform_color([0.5,0.5,0.5])
        # o3d.visualization.draw_geometries([middle_rotate_area, yuandian])

        index = 0
        every_dis = (max(align_point_1[0], align_point_2[0]) - bottom_x) / 50
        add_volum = 0

        for i in range(49):
            every_volum = self.compute_part_cloud_volum(middle_rotate_area, max(align_point_1[0], align_point_2[0])-every_dis*(i+1),
                                                        max(align_point_1[0], align_point_2[0]) - every_dis*i)
            add_volum += every_volum
            if add_volum >= volum:
                index = i
                break
        if index == 0:
            index = 50

        total_dis = every_dis * index

        print '正向 total dis', total_dis
        print "正向 volume:", add_volum

        end_rotate_location_x = max(align_point_1[0], align_point_2[0]) - index * every_dis

        start_rotate_location_x = align_point_1[0]

        cur_add_volume = 0
        if add_volum < volum:
            print 'reverse extension'
            remain_volume = volum - add_volum
            reverse_every_dis = (up_x - max(align_point_1[0], align_point_2[0])) / 50

            reverse_index = 0
            for i in range(49):
                reverse_every_volume = self.compute_part_cloud_volum(middle_rotate_area,
                                                            max(align_point_1[0], align_point_2[0]) + reverse_every_dis * i,
                                                            max(align_point_1[0], align_point_2[0]) + reverse_every_dis * (i+1))
                cur_add_volume += reverse_every_volume
                if cur_add_volume > remain_volume:
                    reverse_index = i
                    break

            start_rotate_location_x = align_point_1[0] + reverse_every_dis * reverse_index

        total_volume = add_volum + cur_add_volume
        for i in range(len(middle_rotate_area.points)):
            if end_rotate_location_x < middle_rotate_area.points[i][0] < start_rotate_location_x:
                final_middle_rotate_area.points.append(middle_rotate_area.points[i])

        final_middle_rotate_area.paint_uniform_color([0.5, 0.02, 1])
        dig_move_pass_points = []
        for i in range(len(final_middle_rotate_area.points)):
            if abs(final_middle_rotate_area.points[i][1] - align_point_1[1]) < 0.005:
                final_middle_rotate_area.colors[i] = [1,0.5,0.8]
                move_pass_point = self.rotate_by_axis_z(final_middle_rotate_area.points[i], theta)
                dig_move_pass_points.append(move_pass_point)

        end_location = [10, 0, 0]
        start_location = [-10, 0, 0]
        for i in range(len(dig_move_pass_points)):
            if dig_move_pass_points[i][0] < end_location[0]:
                end_location = dig_move_pass_points[i]
            if dig_move_pass_points[i][0] > start_location[0]:
                start_location = dig_move_pass_points[i]

        middle_area = self.pcd_rot(final_middle_rotate_area, 0, 0, theta)

        # up_half_x = x_boundary[0] + (x_boundary[1] - x_boundary[0]) / 2
        # if start_location[0] > up_half_x and end_location[0] > up_half_x:
        #     print '上半区'
        #     for i in range(len(middle_rotate_area.points)):
        #         if bottom_x < middle_rotate_area.points[i][0] < start_rotate_location_x:
        #             up_half_final_middle_rotate_area.points.append(middle_rotate_area.points[i])
        #
        #     up_half_final_middle_rotate_area.paint_uniform_color([0.5, 0.02, 1])
        #
        #     up_half_dig_move_pass_points = []
        #     for i in range(len(up_half_final_middle_rotate_area.points)):
        #         if abs(up_half_final_middle_rotate_area.points[i][1] - align_point_1[1]) < 0.005:
        #             up_half_final_middle_rotate_area.colors[i] = [1, 0.5, 0.8]
        #             move_pass_point = self.rotate_by_axis_z(up_half_final_middle_rotate_area.points[i], theta)
        #             up_half_dig_move_pass_points.append(move_pass_point)
        #
        #     up_half_end_location = [10, 0, 0]
        #     up_half_start_location = [-10, 0, 0]
        #     for i in range(len(up_half_dig_move_pass_points)):
        #         if up_half_dig_move_pass_points[i][0] < up_half_end_location[0]:
        #             up_half_end_location = up_half_dig_move_pass_points[i]
        #         if up_half_dig_move_pass_points[i][0] > up_half_start_location[0]:
        #             up_half_start_location = up_half_dig_move_pass_points[i]
        #
        #     up_half_middle_area = self.pcd_rot(up_half_final_middle_rotate_area, 0, 0, theta)
        #
        #     end_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=up_half_end_location)
        #     start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=up_half_start_location)
        #     # o3d.visualization.draw_geometries([up_half_middle_area, yuandian, end_coor, start_coor])
        #
        #     return up_half_middle_area, up_half_dig_move_pass_points, up_half_start_location, up_half_end_location, total_volume, real_ave_depth


        end_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=end_location)
        start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=start_location)
        # o3d.visualization.draw_geometries([middle_area, yuandian, end_coor, start_coor])

        return middle_area, dig_move_pass_points, start_location, end_location, total_volume, real_ave_depth




    def compute_part_cloud_volum(self, cloud, low, high):
        total_depth = 0

        len_points = len(cloud.points)
        density = 31 * 46 * 1.0 / len_points  #每个点所占面积
        for i in range(len(cloud.points)):
            if low <= cloud.points[i][0] <= high:
                total_depth = (self.empty_depth - cloud.points[i][2]) * 100 + total_depth

        volum = total_depth * density
        return volum

    def rotate_by_axis_z(self, location, theta):
        new_location_x = location[0] * math.cos(theta) - location[1] * math.sin(theta)
        new_location_y = location[0] * math.sin(theta) + location[1] * math.cos(theta)
        new_location_z = location[2]
        return (new_location_x, new_location_y, new_location_z)

    def rotate_by_axis_y(self, location, theta):
        new_location_x = location[0] * math.cos(theta) + location[2] * math.sin(theta)
        new_location_y = location[1]
        new_location_z = location[0] * (-1) * math.sin(theta) + location[2] * math.cos(theta)
        return (new_location_x, new_location_y, new_location_z)

#
# if __name__ == '__main__':
#     final_judge_dish_location_qt()


