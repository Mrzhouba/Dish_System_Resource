#!/usr/bin/env python
#coding=UTF-8
import rospy
import moveit_commander
import sys, math, copy
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from final_trajectory_plan import final_trajectory_plan
from recognize_start_loc import recognize_marker
import pyrealsense2 as rs
from sensor_msgs.msg import JointState,Image,CameraInfo, PointCloud2
import open3d as o3d
import Variable
import numpy as np
import numpy
from open3d_ros_helper import open3d_ros_helper as orh
import tf, time, json
import argparse


ur10_pull_ready = Variable.ur10_pull_ready
UR10_Q1_to_left = Variable.UR10_Q1_to_left
# UR10_Q1 = [-0.16467219987978154, -1.430373493825094, 0.9728174209594727, -1.656503979359762, -1.4693554083453577, -0.9290836493121546]

parser = argparse.ArgumentParser(description='UR10 dig dish paraamenters')
# parser.add_argument('--model_name', type=str, default='DGCNN')
# parser.add_argument('--weight_path', type=str, default='/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pointnet2_pytorch/'
#                                                     'dgcnn_log/dgcnn_all_dish_300/models/epoch_280_acc_0.872781_iou_0.753227.pth')
# parser.add_argument('--channels', type=int, default=3)
# parser.add_argument('--nclasses', type=int, default=3)
parser.add_argument('--camera_angele', type=float, default=Variable.camera_angele)
parser.add_argument('--empty_depth', type=float, default=Variable.empty_depth)
parser.add_argument('--ur10_pull_dish_tra_path', type=str, default=Variable.ur10_pull_dish_tra_path)
parser.add_argument('--volume_goal', type=float, default=Variable.require_volume)
parser.add_argument('--max_depth', type=float, default=Variable.empty_depth - Variable.real_max_depth)

dig_dish_args = parser.parse_args()

ur10_pull_to_initial_plan = Variable.ur10_pull_to_initial_plan

# get_model_output = get_model_output(dig_dish_args.model_name, dig_dish_args.channels, dig_dish_args.nclasses, dig_dish_args.weight_path)
final_trajectory_plan = final_trajectory_plan()
# recognize_marker = recognize_marker()

cut_safe_area_xbound_min = Variable.cut_safe_area_xbound_min
cut_safe_area_xbound_max = Variable.cut_safe_area_xbound_max
cut_safe_area_ybound_min = Variable.cut_safe_area_ybound_min
cut_safe_area_ybound_max = Variable.cut_safe_area_ybound_max
cut_safe_area_z = Variable.cut_safe_area_z


class final_judge_dish_location():
    def __init__(self):
        self.test = False

        if self.test == True:
            self.is_save_cloud = False
            self.is_dig = False
        else:
            self.is_save_cloud = True
            self.is_dig = True

        self.is_normalize = False
        self.ur10_pull_dish_plan = self.read_tra_message(dig_dish_args.ur10_pull_dish_tra_path)
        self.ur10_pull_to_initial_plan = self.read_tra_message(ur10_pull_to_initial_plan)



    def points_callback(self, data, args):
        ur10_arm_spoon = args[0]
        ur10_spoon_end_effector_link = args[1]
        signal_ur10_send  = args[2]
        signal_ur3_receive = args[3]
        get_model_output = args[4]

        original_pointcloud = orh.rospc_to_o3dpc(data)
        rotate_pointcloud = self.pcd_rot(original_pointcloud, 0, dig_dish_args.camera_angele, 0)
        yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])

        test_pointcloud = self.cut_safe_area_pointcloud_2200(rotate_pointcloud)
        cal_volum_pointcloud = self.cut_safe_area_pointcloud_5000(rotate_pointcloud)

        cloud_point_data = np.asarray(test_pointcloud.points)
        x_boundary = (np.min(cloud_point_data[:, 0]), np.max(cloud_point_data[:, 0]))
        y_boundary = (np.min(cloud_point_data[:, 1]), np.max(cloud_point_data[:, 1]))

        check_push_pointcloud = test_pointcloud.voxel_down_sample(voxel_size=0.02)


        push_index = self.judge_need_push(check_push_pointcloud, x_boundary, y_boundary, 0.025)
        print push_index
        if push_index != None:
            print 'initial push'
            action_name = 'initial push'
            waypoints = push_index
            orientation_waypoints = None
            if self.is_dig == True:
                is_completed = final_trajectory_plan.push_action_trajectory_plan(action_name, waypoints,
                                                                                    orientation_waypoints,
                                                                                    ur10_arm_spoon,
                                                                                    ur10_spoon_end_effector_link)
                return is_completed
        else:
            points = np.asarray(test_pointcloud.points, dtype=float)
            points_color = np.asarray(test_pointcloud.colors, dtype=float)
            choice = np.random.choice(len(points), 2500, replace=True)
            xyz_points = points[choice, :]
            xyz_points_color = points_color[choice, :]

            cal_volum_points = np.asarray(cal_volum_pointcloud.points, dtype=float)
            cal_volum_points_color = np.asarray(cal_volum_pointcloud.colors, dtype=float)

            #归一划
            if self.is_normalize == True:
                normalize_points = self.data_normalize(points)
                # change 2500 points
                # choice = np.random.choice(len(normalize_points), 2500, replace=True)
                # input_model_data = normalize_points[choice, :]
                input_model_data = normalize_points

            else:
                input_model_data = points


            npoints_pointcloud = o3d.geometry.PointCloud()
            npoints_pointcloud.points = o3d.utility.Vector3dVector(points)
            npoints_pointcloud.colors = o3d.utility.Vector3dVector(points_color)
            # npoints_pointcloud.paint_uniform_color([0.5,0.5,0.5])
            # o3d.visualization.draw_geometries([npoints_pointcloud, yuandian])

            cal_volum_npoints_pointcloud = o3d.geometry.PointCloud()
            cal_volum_npoints_pointcloud.points = o3d.utility.Vector3dVector(cal_volum_points)
            cal_volum_npoints_pointcloud.colors = o3d.utility.Vector3dVector(cal_volum_points_color)

            # 得到模型输出
            model_result = get_model_output.get_result(input_model_data)

            coloring_pointcloud, dig_area_pointcloud, analyze_start_location, analyze_end_location,\
            dig_move_pass_points, action_name, volume = self.analyze_model_reuslt(npoints_pointcloud, model_result, cal_volum_npoints_pointcloud, x_boundary)


            if dig_area_pointcloud is not None and action_name != 'corner push' and volume > 2700:
                start_location, end_location = self.check_point_safely(x_boundary, y_boundary, analyze_start_location,
                                                                       analyze_end_location)

                # print start_location, end_location
                # volume = self.compute_cloud_volume(dig_area_pointcloud)
                print "总体积:" + str(volume)

                # 存储点云
                if self.is_save_cloud == True:
                    self.save_pointcloud(rotate_pointcloud)

                real_coloring_pointcloud = self.pcd_rot(coloring_pointcloud, 0, -dig_dish_args.camera_angele, 0)
                real_dig_area_pointcloud = self.pcd_rot(dig_area_pointcloud, 0, -dig_dish_args.camera_angele, 0)
                real_start_location = self.rotate_by_axis_y(start_location, -dig_dish_args.camera_angele)
                real_end_location = self.rotate_by_axis_y(end_location, -dig_dish_args.camera_angele)


                dig_type = self.judge_dig_action(dig_area_pointcloud, 0.03, 0.4)

                if action_name == 'dig dish':
                    print action_name
                    print dig_type
                    waypoints, orientation_waypoints = self.return_waypoints(dig_type, start_location, end_location, dig_move_pass_points)
                    # print waypoints[-1]
                    # print orientation_waypoints

                    if self.is_dig == True:
                        is_completed, move_dish_plan = final_trajectory_plan.push_action_trajectory_plan(
                            action_name, waypoints, orientation_waypoints, ur10_arm_spoon, ur10_spoon_end_effector_link)

                        # real dig dish
                        if is_completed == True and move_dish_plan != None:
                            signal_ur10_send.send("get to pull loc")
                            ur10_arm_spoon.execute(move_dish_plan)

                            if signal_ur3_receive.recv() == "bowl is ready":
                                signal_ur10_send.send("start pull dish")

                                ur10_arm_spoon.execute(self.ur10_pull_dish_plan)

                                # signal_ur10_send.send("ur10 pull finish")
                                rospy.sleep(0.1)

                                ur10_arm_spoon.execute(self.ur10_pull_to_initial_plan)

                                return is_completed

                        else:
                            ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
                            ur10_arm_spoon.go()
                            return False



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



                elif action_name == 'push dish':
                    print "push dish"
                    waypoints, orientation_waypoints = self.return_push_waypoints(start_location, end_location,
                                                                             dig_move_pass_points)
                    if self.is_dig == True:
                        is_completed = final_trajectory_plan.push_action_trajectory_plan(action_name, waypoints,
                                                                                             orientation_waypoints,
                                                                                             ur10_arm_spoon,
                                                                                             ur10_spoon_end_effector_link)
                        return is_completed

                    if self.test == True:
                        push_start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[0])

                        push_middle_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[1])

                        push_end_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[2])

                        real_start_x = self.get_push_rotate_angle(orientation_waypoints[1], orientation_waypoints[0])

                        p = push_start_coor.get_rotation_matrix_from_yxz((math.pi/2-dig_dish_args.camera_angele, math.pi/2+real_start_x, -math.pi/2))

                        push_start_coor.rotate(p)
                        push_middle_coor.rotate(p)
                        push_end_coor.rotate(p)

                        o3d.visualization.draw_geometries([real_dig_area_pointcloud,  real_coloring_pointcloud, yuandian, push_start_coor, push_middle_coor, push_end_coor])

            else:
                waypoints = self.judge_need_push(check_push_pointcloud, x_boundary, y_boundary, 0.015)
                action_name = 'initial push'
                orientation_waypoints = None
                if self.is_dig == True:
                    if waypoints != None:
                        is_completed = final_trajectory_plan.push_action_trajectory_plan(action_name, waypoints, orientation_waypoints,
                                                                                                ur10_arm_spoon,
                                                                                                ur10_spoon_end_effector_link)
                        return is_completed
                    else:
                        return False

                if self.test == True:
                    push_start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[0])

                    push_middle_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[1])

                    push_end_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[2])

                    real_npoints_pointcloud = self.pcd_rot(npoints_pointcloud, 0, -dig_dish_args.camera_angele, 0)

                    o3d.visualization.draw_geometries( [real_npoints_pointcloud, yuandian, push_start_coor,
                         push_middle_coor, push_end_coor])


    def judge_need_push(self, cloud, x_boundary, y_boundary, tolerance):
        total = 0
        for i in range(len(cloud.points)):
            total = total + dig_dish_args.empty_depth - cloud.points[i][2]
        aver_total = total * 1.0 / len(cloud.points)
        if aver_total > 0.045 and tolerance != None:
            max_index = None
        else:
            x_three = []
            y_three = []
            for i in range(4):
                x_three.append(i * (x_boundary[1]-x_boundary[0]) / 3 + x_boundary[0])
                y_three.append(i * (y_boundary[1] - y_boundary[0]) / 3 + y_boundary[0])

            center_points_depth = 0
            center_num = 0
            four_corner_points_depth = [0,0,0,0]
            four_corner_num = [0,0,0,0]
            for i in range(len(cloud.points)):
                if cloud.points[i][0] > x_three[2] and cloud.points[i][1] < y_three[1]:
                    four_corner_points_depth[0] = four_corner_points_depth[0] + (dig_dish_args.empty_depth - cloud.points[i][2])
                    four_corner_num[0] = four_corner_num[0] + 1
                elif cloud.points[i][0] > x_three[2] and cloud.points[i][1] > y_three[2]:
                    four_corner_points_depth[1] = four_corner_points_depth[1] + (dig_dish_args.empty_depth - cloud.points[i][2])
                    four_corner_num[1] = four_corner_num[1] + 1
                elif cloud.points[i][0] < x_three[1] and cloud.points[i][1] > y_three[2]:
                    four_corner_points_depth[2] = four_corner_points_depth[2] + (dig_dish_args.empty_depth - cloud.points[i][2])
                    four_corner_num[2] = four_corner_num[2] + 1
                elif cloud.points[i][0] < x_three[1] and cloud.points[i][1] < y_three[1]:
                    four_corner_points_depth[3] = four_corner_points_depth[3] + (dig_dish_args.empty_depth - cloud.points[i][2])
                    four_corner_num[3] = four_corner_num[3] + 1

                elif x_three[1] < cloud.points[i][0] < x_three[2] and y_three[1] < cloud.points[i][1] < y_three[2]:
                    center_points_depth = center_points_depth + (dig_dish_args.empty_depth - cloud.points[i][2])
                    center_num = center_num +1

            center_ave_depth = center_points_depth * 1.0 / center_num
            print center_ave_depth
            satisfied_target_area = []
            satisfied_target_area_index = []
            for i in range(4):
                aver_depth = four_corner_points_depth[i] * 1.0 / four_corner_num[i]
                if i == 0 and aver_depth > 0.04:
                    aver_depth = aver_depth + 0.01
                if i == 2 and aver_depth > 0.035:
                    aver_depth = aver_depth - 0.01
                print aver_depth
                if tolerance == None:
                    satisfied_target_area.append(aver_depth)
                else:
                    if aver_depth - center_ave_depth > tolerance:
                        satisfied_target_area.append(aver_depth)
                        satisfied_target_area_index.append(i)
                    else:
                        satisfied_target_area.append(aver_depth)
            # print satisfied_target_area
            # if len(satisfied_target_area_index) != 0:
            if tolerance == None:
                max_index = satisfied_target_area.index(max(satisfied_target_area))
            else:
                if len(satisfied_target_area_index) != 0:
                    max_index = satisfied_target_area.index(max(satisfied_target_area))
                else:
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
        real_push_start_location = self.rotate_by_axis_y(push_start_point, -dig_dish_args.camera_angele)
        waypoints.append(real_push_start_location)
        real_start_push_point = self.rotate_by_axis_y(end_point, -dig_dish_args.camera_angele)
        orientation_waypoints.append(real_start_push_point)


        push_middle_point = order_move_pass_points[int(num/5)]
        deal_push_middle_point = [push_middle_point[0], push_middle_point[1], dig_dish_args.empty_depth-0.06]
        real_push_middle_location = self.rotate_by_axis_y(deal_push_middle_point, -dig_dish_args.camera_angele)
        waypoints.append(real_push_middle_location)

        push_end_point = [start_point[0], start_point[1], dig_dish_args.empty_depth - 0.05]
        real_push_end_location = self.rotate_by_axis_y(push_end_point, -dig_dish_args.camera_angele)
        waypoints.append(real_push_end_location)
        real_end_push_point = self.rotate_by_axis_y(start_point, -dig_dish_args.camera_angele)
        orientation_waypoints.append([real_end_push_point[0], real_end_push_point[1], real_start_push_point[2]])

        return waypoints, orientation_waypoints

    def return_waypoints(self, dig_type, start_point, end_point, move_pass_points, volum, cal_volum_pointcloud, x_boundary, dig_area_pointcloud):
        np_move_pass_points = np.asarray(move_pass_points)
        if end_point[1] - start_point[1] > 0:
            order_move_pass_points = np_move_pass_points[np_move_pass_points[:,1].argsort()]
        else:
            order_move_pass_points = np_move_pass_points[np.argsort(-np_move_pass_points[:,1]), :]

        waypoints = []
        orientation_waypoints = []
        num = len(order_move_pass_points)

        if dig_type == 'full rotate':
            deal_start_point = [start_point[0], start_point[1], start_point[2]-0.055]
            real_start_location = self.rotate_by_axis_y(deal_start_point, -dig_dish_args.camera_angele)
            no_deal_real_start_location = self.rotate_by_axis_y(start_point, -dig_dish_args.camera_angele)
            waypoints.append(real_start_location)
            orientation_waypoints.append(no_deal_real_start_location)

            middle_point = order_move_pass_points[int(num/2)]
            #0.599 0.6 0.535
            #0.529 0.61 0.55

            if middle_point[2] > dig_dish_args.max_depth:
                deal_middle_point = [middle_point[0], middle_point[1], dig_dish_args.max_depth]
            else:
                deal_middle_point = [middle_point[0], middle_point[1], dig_dish_args.max_depth]

            real_middle_location = self.rotate_by_axis_y(deal_middle_point, -dig_dish_args.camera_angele)
            waypoints.append(real_middle_location)

            orientation_middle_location = self.rotate_by_axis_y((middle_point[0], middle_point[1], start_point[2]), -dig_dish_args.camera_angele)
            orientation_waypoints.append(orientation_middle_location)

            end_point_z = self.end_point_z_judge(start_point, end_point, cal_volum_pointcloud, volum, x_boundary,
                                                 dig_area_pointcloud)
            deal_end_point = [end_point[0], end_point[1], end_point_z]
            real_end_location = self.rotate_by_axis_y(deal_end_point, -dig_dish_args.camera_angele)
            no_deal_real_end_location = self.rotate_by_axis_y(deal_end_point, -dig_dish_args.camera_angele)
            waypoints.append(real_end_location)

            orientation_end_location = self.rotate_by_axis_y((end_point[0], end_point[1], start_point[2]), -dig_dish_args.camera_angele)
            orientation_last_location = self.rotate_by_axis_y((end_point[0], end_point[1], 0.5), -dig_dish_args.camera_angele)
            orientation_waypoints.append(orientation_end_location)
            orientation_waypoints.append(orientation_last_location)


        elif dig_type == 'push rotate':
            deal_start_point = [start_point[0], start_point[1], start_point[2] - 0.055]
            real_start_location = self.rotate_by_axis_y(deal_start_point, -dig_dish_args.camera_angele)
            no_deal_real_start_location = self.rotate_by_axis_y(start_point, -dig_dish_args.camera_angele)
            waypoints.append(real_start_location)
            orientation_waypoints.append(no_deal_real_start_location)

            middle_point = order_move_pass_points[int(num / 2)]
            # 0.599 0.6 0.535
            # 0.529 0.61 0.55

            if middle_point[2] > dig_dish_args.max_depth:
                deal_middle_point = [middle_point[0], middle_point[1], dig_dish_args.max_depth]
            else:
                deal_middle_point = [middle_point[0], middle_point[1], dig_dish_args.max_depth]

            real_middle_location = self.rotate_by_axis_y(deal_middle_point, -dig_dish_args.camera_angele)
            waypoints.append(real_middle_location)

            orientation_middle_location = self.rotate_by_axis_y((middle_point[0], middle_point[1], start_point[2]),
                                                                -dig_dish_args.camera_angele)
            orientation_waypoints.append(orientation_middle_location)

            end_point_z = self.end_point_z_judge(start_point, end_point, cal_volum_pointcloud, volum, x_boundary,
                                                 dig_area_pointcloud)
            deal_end_point = [end_point[0], end_point[1], end_point_z]
            real_end_location = self.rotate_by_axis_y(deal_end_point, -dig_dish_args.camera_angele)
            no_deal_real_end_location = self.rotate_by_axis_y(deal_end_point, -dig_dish_args.camera_angele)
            waypoints.append(real_end_location)

            orientation_end_location = self.rotate_by_axis_y((end_point[0], end_point[1], start_point[2]),
                                                             -dig_dish_args.camera_angele)
            orientation_last_location = self.rotate_by_axis_y((end_point[0], end_point[1], 0.5),
                                                              -dig_dish_args.camera_angele)
            orientation_waypoints.append(orientation_end_location)
            orientation_waypoints.append(orientation_last_location)


        return waypoints, orientation_waypoints



    def end_point_z_judge(self, start_point, end_point, cal_volum_pointcloud, volum, x_boundary, dig_area_pointcloud):
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
            total_depth = (dig_dish_args.empty_depth - square_pointcloud.points[i][2]) + total_depth

        ave_depth = total_depth / len(square_pointcloud.points)
        print '终点平均深度', ave_depth
        print 'endpoint_z', end_point[2]

        dig_total_depth = 0
        for i in range(len(dig_area_pointcloud.points)):
            dig_total_depth = (dig_dish_args.empty_depth - dig_area_pointcloud.points[i][2]) + dig_total_depth

        dig_ave_depth = dig_total_depth / len(dig_area_pointcloud.points)
        print '打菜区域平均深度', dig_ave_depth

        up_half_x = x_boundary[0] + (x_boundary[1] - x_boundary[0]) / 2
        if volum < 2500:
            print '体积不到2500'
            end_point_z = dig_dish_args.max_depth
        else:
            if dig_dish_args.empty_depth - dig_ave_depth * 1.8 < dig_dish_args.max_depth:
                print '根据深度'
                end_point_z = dig_dish_args.empty_depth - dig_ave_depth * 1.8
            else:
                end_point_z = dig_dish_args.max_depth

        return end_point_z


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

    def compute_cloud_volume(self, cloud):
        # real_cloud = self.pcd_rot(cloud, 0, -camera_angele, 0)
        density = 31 * 46 *1.0 / 2500
        real_depth = 0
        for i in range(len(cloud.points)):
            depth = (-cloud.points[i][2] + dig_dish_args.empty_depth) * 100
            real_depth = real_depth + depth
        volume = real_depth * density
        return volume

    def get_cloud_corner_loc(self, cloud):
        left_down = 10
        for i in range(len(cloud.points)):
            if (cloud.points[i][2] > 0.5):
                if (cloud.points[i][0] + cloud.points[i][1]) < left_down:
                    left_down = cloud.points[i][0] + cloud.points[i][1]
                    left_down_loc = cloud.points[i]

        right_up = -10
        for i in range(len(cloud.points)):
            if (cloud.points[i][2] > 0.5):
                if (cloud.points[i][0] + cloud.points[i][1]) > right_up:
                    right_up = cloud.points[i][0] + cloud.points[i][1]
                    right_up_loc = cloud.points[i]


        return left_down_loc, right_up_loc

    def check_point_safely(self, x_boundary, y_boundary, analyze_start_location, analyze_end_location):
        start_location = list(analyze_start_location)
        end_location = list(analyze_end_location)

        if start_location[0] - x_boundary[0] < 0.055:
            print "start x outrange"
            start_location[0] = x_boundary[0] + 0.055
        if x_boundary[1] - start_location[0] < 0.055:
            print "start x outrange"
            start_location[0] = x_boundary[1] - 0.055

        if end_location[0] - x_boundary[0] < 0.055:
            print "end x outrange"
            end_location[0] = x_boundary[0] + 0.055
        if x_boundary[1] - end_location[0] < 0.055:
            print "end x outrange"
            end_location[0] = x_boundary[1] - 0.055

        if start_location[1] - y_boundary[0] < 0.055:
            print "start y outrange"
            start_location[1] = y_boundary[0] + 0.055
        if y_boundary[1] - start_location[1] < 0.055:
            print "start y outrange"
            start_location[1] = y_boundary[1] - 0.055

        if end_location[1] - y_boundary[0] < 0.055:
            print "end y outrange"
            end_location[1] = y_boundary[0] + 0.055
        if y_boundary[1] - end_location[1] < 0.055:
            print "end y outrange"
            end_location[1] = y_boundary[1] - 0.055
        return start_location, end_location

    def data_normalize(self, data):
        mean = np.mean(data, axis=0)
        data -= mean
        m = np.max(np.sqrt(np.sum(np.power(data, 2), axis=1)))
        data /= m
        return data

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

    def get_push_rotate_angle(self, start_loc, end_loc):
        delta_x = abs(end_loc[0] - start_loc[0])
        delta_y = abs(end_loc[1] - start_loc[1])

        theta = math.atan2(delta_x, delta_y)
        return theta


    def judge_dig_action(self, middle_area_pointcloud, bottom_threshold, proportion):
        num = len(middle_area_pointcloud.points)
        satisfy_bottom_dis_num = 0
        for i in range(num):

            if (dig_dish_args.empty_depth-middle_area_pointcloud.points[i][2] > bottom_threshold):
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

    #select target point
    def select_points(self, cloud, pred):
        # pred = pred_choice.numpy()

        n = len(pred)
        for i in range(n):
            if (pred[i] == 1):
                cloud.colors[i] = [1,0,0]
            elif (pred[i] == 2):
                cloud.colors[i] = [0,1,0]
            else:
                cloud.colors[i] = [0.5, 0.5, 0.5]
        return cloud

    def add_density(self, cloud, radius):
        cloud_tree = o3d.geometry.KDTreeFlann(cloud)
        num = len(cloud.points)
        density_result = []
        remain_depth = []
        for i in range(num):
            [k, idx, _] = cloud_tree.search_radius_vector_3d(cloud.points[i], radius)
            # if cloud.points[i][2] != 0:
            density_result.append(k * 1.0 / 100)
            remain_depth.append(0.6 - cloud.points[i][2])

        xyz = np.asarray(cloud.points, dtype=float)
        density_result = np.asarray([density_result]).T
        remain_depth = np.asarray([remain_depth]).T

        result_temp = np.hstack((xyz, density_result))
        result = np.hstack((result_temp, remain_depth))
        print result
        return result

    def add_depth(self, xyz_points):
        num = len(xyz_points)
        remain_depth = []
        for i in range(num):
            remain_depth.append(0.6- xyz_points[i][2])
        remain_depth = np.asarray([remain_depth]).T
        result = np.hstack((xyz_points, remain_depth))

        return result

    def analyze_model_reuslt(self, cloud, model_result, cal_volum_npoints_pointcloud, x_boundary):
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
            else:cloud.colors[i] = origin_color[i]

        start_pointcloud.points = o3d.utility.Vector3dVector(start_points)
        end_pointcloud.points = o3d.utility.Vector3dVector(end_points)

        print 'start_pointcloud点数', len(start_pointcloud.points)
        print 'end_pointcloud点数', len(end_pointcloud.points)

        start_center_point = start_pointcloud.get_center()
        end_center_point = end_pointcloud.get_center()

        if start_center_point[1] - end_center_point[1] < -0.01 and start_center_point[0] - end_center_point[0] > 0.01:
            action_name = 'dig dish'
            require_volum = dig_dish_args.volume_goal
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
            middle_area_pointcloud, dig_move_pass_points, start_location, end_location, volume = self.get_required_volum_area(start_center_point,
                                                                                                                              end_center_point,
                                                                                                                              0.1,
                                                                                                                              require_volum,
                                                                                                                              cal_volum_npoints_pointcloud,
                                                                                                                              x_boundary)

            # middle_area_pointcloud_tree = o3d.geometry.KDTreeFlann(middle_area_pointcloud)
            # [_, start_idx, _] = middle_area_pointcloud_tree.search_knn_vector_3d(end_location, int(len(middle_area_pointcloud.points)/5))
            # end_pointset = np.asarray(middle_area_pointcloud.points)[start_idx[1:], :]
            # end_location = [end_location[0], end_location[1], np.mean(end_pointset, 0)[2]]

            if volume > 2700:
                return cloud, middle_area_pointcloud, start_location, end_location, dig_move_pass_points, action_name, volume
            else:
                print '体积不到2700'
                return None, None, None, None, None, 'corner push', None
        else:
            return None, None, None, None, None, 'corner push', None


    def get_required_volum_area(self, cloud, point_1, point_2, area_width, volum, cal_volum_npoints_pointcloud, x_boundary):
        yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
        middle_rotate_area = o3d.geometry.PointCloud()
        up_half_final_middle_rotate_area = o3d.geometry.PointCloud()
        final_middle_rotate_area = o3d.geometry.PointCloud()

        middle_rotate_area_points = []
        theta = math.atan2((point_1[1] - point_2[1]), (point_1[0] - point_2[0]))
        # o3d.visualization.draw_geometries([cloud, yuandian])

        align_cal_volum_cloud = self.pcd_rot(cal_volum_npoints_pointcloud, 0, 0, -theta)
        # o3d.visualization.draw_geometries([align_cal_volum_cloud, yuandian])

        align_point_1 = self.rotate_by_axis_z(point_1, -theta)  # start
        align_point_2 = self.rotate_by_axis_z(point_2, -theta)
        # print align_point_1, align_point_2
        all_dig_move_pass_points = []

        for i in range(len(align_cal_volum_cloud.points)):
            if align_point_1[1] - 0.005 < align_cal_volum_cloud.points[i][1] < align_point_1[1] + 0.005:
                all_dig_move_pass_points.append(align_cal_volum_cloud.points[i])

            if align_point_2[1] - area_width / 2 < align_cal_volum_cloud.points[i][1] < align_point_2[
                1] + area_width / 2:
                # align_cloud.colors[i] = [0.5,0,0.1]
                middle_rotate_area_points.append(align_cal_volum_cloud.points[i])

        np_dig_move_pass_points = np.asarray(all_dig_move_pass_points)
        order_move_pass_points = np_dig_move_pass_points[np_dig_move_pass_points[:, 0].argsort()]
        up_x = order_move_pass_points[-1][0]
        bottom_x = order_move_pass_points[0][0]

        middle_rotate_area.points = o3d.utility.Vector3dVector(middle_rotate_area_points)
        middle_rotate_area.paint_uniform_color([0.5, 0.5, 0.5])
        # o3d.visualization.draw_geometries([middle_rotate_area, yuandian])

        index = 0
        every_dis = (max(align_point_1[0], align_point_2[0]) - bottom_x) / 50
        add_volum = 0

        for i in range(49):
            every_volum = self.compute_part_cloud_volum(middle_rotate_area,
                                                        max(align_point_1[0], align_point_2[0]) - every_dis * (i + 1),
                                                        max(align_point_1[0], align_point_2[0]) - every_dis * i)
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
                                                                     max(align_point_1[0],
                                                                         align_point_2[0]) + reverse_every_dis * i,
                                                                     max(align_point_1[0],
                                                                         align_point_2[0]) + reverse_every_dis * (
                                                                                 i + 1))
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
                final_middle_rotate_area.colors[i] = [1, 0.5, 0.8]
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

        up_half_x = x_boundary[0] + (x_boundary[1] - x_boundary[0]) / 2
        if start_location[0] > up_half_x and end_location[0] > up_half_x:
            print '上半区'
            for i in range(len(middle_rotate_area.points)):
                if bottom_x < middle_rotate_area.points[i][0] < start_rotate_location_x:
                    up_half_final_middle_rotate_area.points.append(middle_rotate_area.points[i])

            up_half_final_middle_rotate_area.paint_uniform_color([0.5, 0.02, 1])

            up_half_dig_move_pass_points = []
            for i in range(len(up_half_final_middle_rotate_area.points)):
                if abs(up_half_final_middle_rotate_area.points[i][1] - align_point_1[1]) < 0.005:
                    up_half_final_middle_rotate_area.colors[i] = [1, 0.5, 0.8]
                    move_pass_point = self.rotate_by_axis_z(up_half_final_middle_rotate_area.points[i], theta)
                    up_half_dig_move_pass_points.append(move_pass_point)

            up_half_end_location = [10, 0, 0]
            up_half_start_location = [-10, 0, 0]
            for i in range(len(up_half_dig_move_pass_points)):
                if up_half_dig_move_pass_points[i][0] < up_half_end_location[0]:
                    up_half_end_location = up_half_dig_move_pass_points[i]
                if up_half_dig_move_pass_points[i][0] > up_half_start_location[0]:
                    up_half_start_location = up_half_dig_move_pass_points[i]

            up_half_middle_area = self.pcd_rot(up_half_final_middle_rotate_area, 0, 0, theta)

            end_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=up_half_end_location)
            start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=up_half_start_location)
            # o3d.visualization.draw_geometries([up_half_middle_area, yuandian, end_coor, start_coor])

            return up_half_middle_area, up_half_dig_move_pass_points, up_half_start_location, up_half_end_location, total_volume

        end_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=end_location)
        start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=start_location)
        # o3d.visualization.draw_geometries([middle_area, yuandian, end_coor, start_coor])

        return middle_area, dig_move_pass_points, start_location, end_location, total_volume


    def four_corner_push(self, complete_cloud, x_boundary, y_boundary):

        x_boundary = [x_boundary[0] + 0.05, x_boundary[1] - 0.05]
        y_boundary = [y_boundary[0] + 0.05, y_boundary[1] - 0.05]
        four_corner_loc = [[x_boundary[1], y_boundary[0]], [x_boundary[1], y_boundary[1]],
                           [x_boundary[0], y_boundary[1]], [x_boundary[0], y_boundary[0]]]

        x_boundary_copy = [x_boundary[0] + 0.03, x_boundary[1] - 0.03]
        y_boundary_copy = [y_boundary[0] + 0.03, y_boundary[1] - 0.03]
        four_corner_loc_copy = [[x_boundary_copy[1], y_boundary_copy[0]], [x_boundary_copy[1], y_boundary_copy[1]],
                           [x_boundary_copy[0], y_boundary_copy[1]], [x_boundary_copy[0], y_boundary_copy[0]]]

        cloud_center_loc = [(x_boundary[1]+x_boundary[0])/2, (y_boundary[0]+y_boundary[1])/2]
        left_up_volum=0
        right_up_volum = 0
        right_down_volum = 0
        left_down_volum = 0
        left_up_points = []
        right_up_points = []
        right_down_points = []
        left_down_points = []
        for i in range(len(complete_cloud.points)):
            if complete_cloud.points[i][0] > cloud_center_loc[0] and complete_cloud.points[i][1] < cloud_center_loc[1]:
                left_up_volum = left_up_volum + (dig_dish_args.empty_depth - complete_cloud.points[i][2]) * 100
            elif complete_cloud.points[i][0] > cloud_center_loc[0] and complete_cloud.points[i][1] > cloud_center_loc[1]:
                right_up_volum = right_up_volum + (dig_dish_args.empty_depth - complete_cloud.points[i][2]) * 100
            elif complete_cloud.points[i][0] < cloud_center_loc[0] and complete_cloud.points[i][1] > cloud_center_loc[1]:
                right_down_volum = right_down_volum + (dig_dish_args.empty_depth - complete_cloud.points[i][2]) * 100
            elif complete_cloud.points[i][0] < cloud_center_loc[0] and complete_cloud.points[i][1] < cloud_center_loc[1]:
                left_down_volum = left_down_volum + (dig_dish_args.empty_depth - complete_cloud.points[i][2]) * 100

            if four_corner_loc_copy[0][0]<complete_cloud.points[i][0] < four_corner_loc[0][0] and four_corner_loc[0][1]<complete_cloud.points[i][1] < four_corner_loc_copy[0][1]:
                left_up_points.append(complete_cloud.points[i])
            if four_corner_loc_copy[1][0]<complete_cloud.points[i][0] < four_corner_loc[1][0] and four_corner_loc_copy[1][1]<complete_cloud.points[i][1] < four_corner_loc[1][1]:
                right_up_points.append(complete_cloud.points[i])
            if four_corner_loc[2][0]<complete_cloud.points[i][0] < four_corner_loc_copy[2][0] and four_corner_loc_copy[2][1]<complete_cloud.points[i][1] < four_corner_loc[2][1]:
                right_down_points.append(complete_cloud.points[i])
            if four_corner_loc[3][0]<complete_cloud.points[i][0] < four_corner_loc_copy[3][0] and four_corner_loc[3][1]<complete_cloud.points[i][1] < four_corner_loc_copy[3][1]:
                left_down_points.append(complete_cloud.points[i])

        four_area_list = [left_up_volum, right_up_volum, right_down_volum, left_down_volum]

        target_area_index = four_area_list.index(max(four_area_list))
        print (target_area_index)

        target_push_start = [0,0,10]
        if target_area_index == 0:
            for i in range(len(left_up_points)):
                if left_up_points[i][2] < target_push_start[2]:
                    target_push_start = left_up_points[i]
        if target_area_index == 1:
            for i in range(len(right_up_points)):
                if right_up_points[i][2] < target_push_start[2]:
                    target_push_start = right_up_points[i]
        if target_area_index == 2:
            for i in range(len(right_down_points)):
                if right_down_points[i][2] < target_push_start[2]:
                    target_push_start = right_down_points[i]
        if target_area_index == 3:
            for i in range(len(left_down_points)):
                if left_down_points[i][2] < target_push_start[2]:
                    target_push_start = left_down_points[i]

        waypoints = []
        deal_target_push_start = [target_push_start[0], target_push_start[1], target_push_start[2]-0.05]
        real_target_push_start = self.rotate_by_axis_y(deal_target_push_start, -dig_dish_args.camera_angele)
        waypoints.append(real_target_push_start)

        yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0,0,0])
        print target_push_start

        if target_area_index == 0:
            target_push_middle_x = max(target_push_start[0], cloud_center_loc[0]) - ( abs(target_push_start[0] - cloud_center_loc[0]) * 1.0 / 5)
            target_push_middle_y = min(target_push_start[1], cloud_center_loc[1]) + (abs(target_push_start[1] - cloud_center_loc[1])*1.0/5)
        elif target_area_index == 1:
            target_push_middle_x = max(target_push_start[0], cloud_center_loc[0]) - ( abs(target_push_start[0] - cloud_center_loc[0]) * 1.0 / 5)
            target_push_middle_y = max(target_push_start[1], cloud_center_loc[1]) - ( abs(target_push_start[1] - cloud_center_loc[1]) * 1.0 / 5)
        elif target_area_index == 2:
            target_push_middle_x = min(target_push_start[0], cloud_center_loc[0]) + ( abs(target_push_start[0] - cloud_center_loc[0]) * 1.0 / 5)
            target_push_middle_y = max(target_push_start[1], cloud_center_loc[1]) - ( abs(target_push_start[1] - cloud_center_loc[1]) * 1.0 / 5)
        elif target_area_index == 3:
            target_push_middle_x = min(target_push_start[0], cloud_center_loc[0]) + ( abs(target_push_start[0] - cloud_center_loc[0]) * 1.0 / 5)
            target_push_middle_y = min(target_push_start[1], cloud_center_loc[1]) + ( abs(target_push_start[1] - cloud_center_loc[1]) * 1.0 / 5)

        target_push_middle = [target_push_middle_x, target_push_middle_y, dig_dish_args.empty_depth-0.06]
        real_target_push_middle = self.rotate_by_axis_y(target_push_middle, -dig_dish_args.camera_angele)
        waypoints.append(real_target_push_middle)

        deal_target_push_end = [cloud_center_loc[0], cloud_center_loc[1], dig_dish_args.empty_depth - 0.055]
        real_target_push_end = self.rotate_by_axis_y(deal_target_push_end, -dig_dish_args.camera_angele)
        waypoints.append(real_target_push_end)

        rotate_message = []
        theta = math.atan2(abs(target_push_start[0] - cloud_center_loc[0]), abs(target_push_start[1] - cloud_center_loc[1]))
        if target_area_index == 0 or target_area_index == 2:
            rotate_message = [[math.pi/2, -math.pi/2, -math.pi/2-theta, -math.pi/2], 'zyxz']
        elif target_area_index == 1 or target_area_index == 3:
            rotate_message = [[math.pi / 2, -math.pi / 2, math.pi / 2 + theta, -math.pi / 2], 'zyxz']

        return waypoints, rotate_message


    def Intercept_designated_area(self, cloud, point_1, point_2, area_width):
        # yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
        middle_rotate_area = o3d.geometry.PointCloud()
        middle_rotate_area_points = []
        theta = math.atan2((point_1[1]-point_2[1]), (point_1[0]-point_2[0]))

        align_cloud = self.pcd_rot(cloud, 0, 0, -theta)
        align_point_1 = self.rotate_by_axis_z(point_1, -theta)
        align_point_2 = self.rotate_by_axis_z(point_2, -theta)
        # print align_point_1, align_point_2
        dig_move_pass_points = []

        for i in range(len(align_cloud.points)):
            if min(align_point_1[0], align_point_2[0]) < align_cloud.points[i][0] < max(align_point_1[0], align_point_2[0]) and \
                    align_point_2[1]-area_width/2 < align_cloud.points[i][1] < align_point_2[1]+area_width/2:
                align_cloud.colors[i] = [0.5,0,0.1]
                middle_rotate_area_points.append(align_cloud.points[i])
                if abs(align_cloud.points[i][1] - align_point_1[1]) < 0.005:
                    align_cloud.colors[i] = [1,0.5,0.8]
                    move_pass_point = self.rotate_by_axis_z(align_cloud.points[i], theta)
                    dig_move_pass_points.append(move_pass_point)

        # print dig_move_pass_point[0]
        middle_rotate_area.points = o3d.utility.Vector3dVector(middle_rotate_area_points)
        # o3d.visualization.draw_geometries([align_cloud, yuandian])
        coloring_cloud = self.pcd_rot(align_cloud, 0, 0, theta)
        middle_area = self.pcd_rot(middle_rotate_area, 0, 0, theta)

        return coloring_cloud, middle_area, dig_move_pass_points

    def compute_part_cloud_volum(self, cloud, low, high):
        total_depth = 0

        len_points = len(cloud.points)
        density = 31 * 46 * 1.0 / len_points  # 每个点所占面积
        for i in range(len(cloud.points)):
            if low <= cloud.points[i][0] <= high:
                total_depth = (dig_dish_args.empty_depth - cloud.points[i][2]) * 100 + total_depth

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




if __name__ == '__main__':

    try:
        final_judge_dish_location()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass

