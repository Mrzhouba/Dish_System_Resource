#!/usr/bin/env python
#coding=UTF-8
import rospy
import moveit_commander
import sys, math, copy
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/pointnet2_pytorch/utils')

from playsound import playsound
import Variable
from metal_push_trajectory_plan_test import metal_push_trajectory_plan_test
from sensor_msgs.msg import JointState,Image,CameraInfo, PointCloud2
import open3d as o3d
import numpy as np
import numpy, json
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import matplotlib.pyplot as plt
from moveit_msgs.msg import RobotTrajectory
from open3d_ros_helper import open3d_ros_helper as orh
from trajectory_msgs.msg import *
import tf, time
import paho.mqtt.client as mqtt
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal
import actionlib
# from moveit_msgs.msg import PlanningScene
import multiprocessing, os
import psutil, serial, binascii

from grippers_control import grippers_control
from serial_drop_bowl import drop_bowl

grippers_control = grippers_control()
drop_bowl = drop_bowl()


HOST = Variable.HOST
PORT = Variable.PORT
car_mqtt_client = mqtt.Client("control_car")
bowl_drop_mqtt_client = mqtt.Client("control_bowl_drop")
convey_mqtt_client = mqtt.Client("control_convey")

base_path = Variable.base_path
weight_path = base_path + 'dgcnn_log/dgcnn_all_dish_300/models/epoch_280_acc_0.872781_iou_0.753227.pth'
# weight_path = base_path + 'dgcnn_log/real_dish/epoch_478_acc_0.844848_iou_0.684396.pth'
# weight_path = Variable.weight_path
# weight_path = '/home/robot/PointCloud/together_dish_dish_outputs_h5/together_dish_h5_dgg/models/epoch_369_acc_0.800710_iou_0.599141.pth'
#
# weight_path = base_path + 'dgg_log/real_dish/epoch_499_acc_0.840018_iou_0.634509.pth'
# weight_path = base_path + 'dgg_log/dgg_resize/models/epoch_275_acc_0.869592_iou_0.739159.pth'
# weight_path = base_path + 'dgg_log/dgg_all_dish_300/models/epoch_280_acc_0.865356_iou_0.739130.pth'
# weight_path = base_path + 'dgg_log/dgg_only_dig_300/models/epoch_265_acc_0.868822_iou_0.768025.pth'

# weight_path = base_path + 'pointnet_log/444_model_0.5223.pth'

model_name = Variable.model_name
channels = Variable.channels
nclasses = Variable.nclasses

metal_push_trajectory_plan_test = metal_push_trajectory_plan_test()
# recognize_marker = recognize_marker()

UR3_Q1 = Variable.UR3_Q1
#old dual-arm-move-together
# UR3_lay_location = [0.7378380298614502, -1.7818334738360804, -1.903487507496969, 0.5219758749008179, 1.3935306072235107, 4.712612152099609]

UR3_lay_location = Variable.UR3_lay_location

# one_bowl_loc = [[0.6785475611686707, -2.143420998250143, -1.6524904409991663, 0.632731556892395, 1.4522337913513184, 4.710690498352051]]
# two_bowl_loc = [[0.5609722137451172, -2.3851164023028772, -1.1958144346820276, 0.4179569482803345, 1.5704615116119385, 4.708253383636475],
#                 [0.854677140712738, -1.9196131865130823, -2.0202696959124964, 0.7756901979446411, 1.2757338285446167, 4.7146172523498535]]

one_bowl__plate_loc = Variable.one_bowl_plate_loc
two_bowl_plate_loc = Variable.two_bowl_plate_loc

one_bowl_up_loc = Variable.one_bowl_up_loc
two_bowl_up_loc = Variable.two_bowl_up_loc

UR10_pull = Variable.UR10_pull
UR10_Q1_to_left = Variable.UR10_Q1_to_left
UR10_Q1 = Variable.UR10_Q1

ur10_max_acceleration = Variable.ur10_max_acceleration
ur10_max_velocity = Variable.ur10_max_velocity

ur3_max_acceleration = Variable.ur3_max_acceleration
ur3_max_velocity = Variable.ur3_max_velocity
# ur10_pull_reay_ur3 = [-0.6907594839679163, -1.3179119269000452, 1.5257153511047363, -2.3891738096820276, -2.442038122807638, 1.6738557815551758]
# ur10_pull_ur3 = [-0.7290824095355433, -1.3175042311297815, 1.5274300575256348, -2.542030159627096, -2.38277250925173, 3.5460495948791504]

camera_angele = Variable.camera_angele
# empty_depth = 0.638144
empty_depth = Variable.empty_depth

volume_goal = Variable.require_volume
max_depth = Variable.empty_depth - Variable.real_max_depth
real_max_depth = Variable.real_max_depth


ur10_pull_dish_tra_path = Variable.ur10_pull_dish_tra_path
ur10_pull_to_initial_plan = Variable.ur10_pull_to_initial_plan


cut_safe_area_xbound_min = Variable.cut_safe_area_xbound_min
cut_safe_area_xbound_max = Variable.cut_safe_area_xbound_max
cut_safe_area_ybound_min = Variable.cut_safe_area_ybound_min
cut_safe_area_ybound_max = Variable.cut_safe_area_ybound_max
cut_safe_area_z = Variable.cut_safe_area_z

pause_usb_name = Variable.pause_usb_name

class model_effect_test():
    def __init__(self):
        fn = sys.stdin.fileno()
        self.is_dig = True

        signal_ur3_send, signal_ur3_receive = multiprocessing.Pipe(True)
        signal_ur10_send, signal_ur10_receive = multiprocessing.Pipe(True)
        signal_stop_send, signal_stop_receive = multiprocessing.Pipe(True)


        dig_dish = multiprocessing.Process(target=self.dig_dish, args=(fn, signal_ur10_send, signal_ur3_receive, signal_stop_receive))
        get_bowl = multiprocessing.Process(target=self.ur3_get_bowl, args=(signal_ur3_send, signal_ur10_receive, signal_stop_receive))
        dig_dish.start()
        get_bowl.start()
        dig_dish_pid = dig_dish.pid
        get_bowl_pid = get_bowl.pid

        pause_process = multiprocessing.Process(target=self.pause_process, args=(dig_dish_pid, get_bowl_pid, signal_stop_send))
        # pause_process.start()
        add_dish_pause_process = multiprocessing.Process(target=self.add_dish_pause_process, args=(dig_dish_pid, get_bowl_pid, signal_stop_send))
        # add_dish_pause_process.start()

        dig_dish.join()
        get_bowl.join()
        # pause_process.join()
        # add_dish_pause_process.join()

    def pause_process(self, dig_dish_pid, get_bowl_pid, signal_stop_receive):
        dish_pause = psutil.Process(dig_dish_pid)
        bowl_pause = psutil.Process(get_bowl_pid)
        portx = pause_usb_name
        bps = 9600
        timex = None
        ser = serial.Serial(portx, bps, timeout=timex)
        while 1:
            data = binascii.b2a_hex(ser.read())
            if data == "13" and self.is_dig is True:
                dish_pause.suspend()
                bowl_pause.suspend()
                print('打菜暂停运行')
                self.is_dig = False

            data = binascii.b2a_hex(ser.read())
            if data == "13" and self.is_dig is False:
                dish_pause.resume()
                bowl_pause.resume()
                print('打菜恢复运行')
                signal_stop_receive.send('resume')
                self.is_dig = True
        # while 1:
        #     if trans_stop.status == 1:
        #         dish_pause.suspend()
        #         bowl_pause.suspend()
        #         print('打菜暂停运行')
        #     if trans_stop.status == 2:
        #         dish_pause.resume()
        #         bowl_pause.resume()
        #         print('打菜恢复运行')
        #     else:
        #         pass

    def add_dish_pause_process(self, dig_dish_pid, get_bowl_pid, signal_stop_send):
        dish_pause = psutil.Process(dig_dish_pid)
        bowl_pause = psutil.Process(get_bowl_pid)
        while 1:
            is_stop_signal = signal_stop_send.recv()
            if is_stop_signal == 'stop':
                dish_pause.suspend()
                bowl_pause.suspend()

                print('打菜暂停运行')

                self.is_dig = False

            elif is_stop_signal == 'resume':
                dish_pause.resume()
                bowl_pause.resume()

                print('打菜恢复运行')
                self.is_dig = True

    def dig_dish(self, fn, signal_ur10_send, signal_ur3_receive, signal_stop_receive):
        from get_model_output import get_model_output
        get_model_output = get_model_output(model_name, channels, nclasses, weight_path)
        sys.stdin = os.fdopen(fn)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("model_effect", anonymous=True, disable_signals=True)
        # self.ur10_scene = PlanningSceneInterface("odom")
        #
        # self.ur3_scene = PlanningSceneInterface("odom", ns="ur3")

        reference_frame = "body_link"
        ur10_group_name = "ur10_with_gripper"
        self.ur10_arm_spoon = moveit_commander.MoveGroupCommander(ur10_group_name, ns='ur10')

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
        self.ur10_arm_spoon.set_max_acceleration_scaling_factor(0.5)
        self.ur10_arm_spoon.set_max_velocity_scaling_factor(0.5)

        # 获取终端link的名称
        self.ur10_spoon_end_effector_link = self.ur10_arm_spoon.get_end_effector_link()

        # self.add_desk_belt()
        self.joints_name = self.ur10_arm_spoon.get_joints()

        self.ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
        self.ur10_arm_spoon.go()

        self.test = False

        if self.test == True:
            self.is_save_cloud = False
            self.is_dig = False
        else:
            self.is_save_cloud = False
            self.is_dig = True

        self.is_normalize = False
        self.monitor_push = []



        self.ur10_pull_dish_plan = self.read_tra_message(ur10_pull_dish_tra_path)
        self.ur10_pull_to_initial_plan = self.read_tra_message(ur10_pull_to_initial_plan)

        # self.add_bowl()
        # rospy.sleep(1.5)
        # self.delete_bowl()

        # self.scene.attachMesh()
        # self.ur10_arm_spoon.attach_object("bowl", "bowl", ["grip_tip_link"])
        # self.scene.attachMesh()
        args = [signal_ur10_send, signal_ur3_receive, get_model_output, signal_stop_receive]
        while not rospy.is_shutdown():
            if self.test == False:
                start_signal = signal_ur3_receive.recv()
            else:
                start_signal = 'dig start'
            completed = False
            is_need_push = False
            if start_signal == 'dig start':
                while not completed:
                    point_cloud_data = rospy.wait_for_message("/camera2/depth/color/points", PointCloud2, timeout=1)
                    completed, is_need_push = self.deal_data(point_cloud_data, args, is_need_push, 'ChaoPuGua')
                    print completed, is_need_push
        # self.point_sub = rospy.Subscriber("/camera2/depth/color/points", PointCloud2, self.points_callback,
        #                                   (signal_ur10_send, signal_ur3_receive, get_model_output))
        # rospy.spin()

    def ur3_get_bowl(self, signal_ur3_send, signal_ur10_receive, signal_stop_receive):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("catch_bowl", anonymous=True, disable_signals=True)
        reference_frame = "body_link"

        # 初始化UR3规划组
        ur3_group_name = "ur3_with_gripper"
        ur3_arm_ag95 = moveit_commander.MoveGroupCommander(ur3_group_name, ns="ur3")
        # 当运动规划失败后，允许重新规划
        ur3_arm_ag95.allow_replanning(False)
        ur3_arm_ag95.set_planning_time(1.0)

        # 设置目标位置所使用的参考坐标系
        ur3_arm_ag95.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        ur3_arm_ag95.set_goal_position_tolerance(0.001)
        ur3_arm_ag95.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        ur3_arm_ag95.set_max_acceleration_scaling_factor(0.5)
        ur3_arm_ag95.set_max_velocity_scaling_factor(0.5)

        # 获取终端link的名称
        ur3_end_effector_link = ur3_arm_ag95.get_end_effector_link()

        bowl_drop_mqtt_client.connect(HOST, PORT, 60)
        bowl_drop_mqtt_client.loop_start()

        convey_mqtt_client.connect(HOST, PORT, 60)
        convey_mqtt_client.loop_start()


        AG95_client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)

        ur3_get_bowl_plan = self.read_tra_message(Variable.ur3_get_bowl_plan)

        ur3_pull_dish_plan = self.read_tra_message(Variable.ur3_pull_dish_plan)

        ur3_get_start_plan = self.read_tra_message(Variable.ur3_get_start_plan)
        ur3_arm_ag95.execute(ur3_get_start_plan)

        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()

        have_plate = False
        need_num_bowl = 4
        bowl_loc, bowl_up_loc = self.generate_bowl_loc(need_num_bowl)
        if have_plate == True:
            drop_bowl.control_convey_belt(-0.87, convey_mqtt_client, False)
        move_num = 0

        for i in range(need_num_bowl):
            if i == 3333 or i == 8888 or i == 131313:
                self.add_bowl_to_device_up(ur3_arm_ag95, AG95_client)

            ur3_arm_ag95.set_joint_value_target(UR3_Q1)
            ur3_arm_ag95.go()


            signal_ur3_send.send("dig start")
            # while 1:
            #     add_stop_signal = signal_ur10_receive.recv()
            #     if add_stop_signal == 'add_dish':
            #         print "add add add"
            #
            #         signal_stop_receive.send('stop')
            #         # resume_signal = signal_stop_receive.recv()
            #         # print resume_signal
            #         # if resume_signal == 2:
            #         #     print 11334455
            #         rospy.sleep(0.5)
            #         signal_ur3_send.send('add_dish_finish')
            #     elif add_stop_signal == 'not_add_dish':
            #         break


            # ur3_arm_ag95.set_joint_value_target(UR3_get_bowl_loction)
            ur3_arm_ag95.execute(ur3_get_bowl_plan)

            rospy.sleep(0.2)

            grippers_control.AG95_gripper(100, 0, AG95_client)
            drop_finish = drop_bowl.send_order_drop_bowl(bowl_drop_mqtt_client)

            if drop_finish == True:
                grippers_control.AG95_gripper(100, 5, AG95_client)
                rospy.sleep(0.2)
                grippers_control.AG95_gripper(100, 0, AG95_client)
                rospy.sleep(0.2)

                ur3_arm_ag95.set_joint_value_target(Variable.ur3_lower_loc)
                ur3_arm_ag95.go()

                dish_dig_finish = signal_ur10_receive.recv()

                if dish_dig_finish == "get to pull loc":
                    ur3_arm_ag95.set_joint_value_target(UR3_lay_location)
                    ur3_arm_ag95.go()

                    signal_ur3_send.send("bowl is ready")
                    spoon_ready = signal_ur10_receive.recv()
                    if spoon_ready == "spoon is ready":
                        ur3_arm_ag95.execute(ur3_pull_dish_plan)

                    start_dish_signal = signal_ur10_receive.recv()
                    if start_dish_signal == "pull dish finish":

                        ur3_arm_ag95.set_joint_value_target(bowl_loc[i])
                        ur3_arm_ag95.go()

                        grippers_control.AG95_gripper(100, 55, AG95_client)

                        rospy.sleep(0.2)
                        ur3_arm_ag95.set_joint_value_target(bowl_up_loc[i])
                        ur3_arm_ag95.go()

                        if i % 2 == 1 and i != need_num_bowl-1:
                            move_num = move_num + 1
                            next_distance = -0.15
                            drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
        if have_plate == True:
            exit_distance = 0.87 + 0.15 * (move_num)
            drop_bowl.control_convey_belt(exit_distance, convey_mqtt_client, False)
        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()


    def deal_data(self, data, args, pre_need_push, current_dish_name):
        # ur10_arm_spoon = args[0]
        # ur10_spoon_end_effector_link = args[1]
        self.can_judge = True
        signal_ur10_send = args[0]
        signal_ur3_receive = args[1]
        get_model_output = args[2]
        signal_stop_receive = args[3]

        original_pointcloud = orh.rospc_to_o3dpc(data)
        rotate_pointcloud = self.pcd_rot(original_pointcloud, 0, camera_angele, 0)
        yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])

        test_pointcloud = self.cut_safe_area_pointcloud_2200(rotate_pointcloud)
        cal_volum_pointcloud = self.cut_safe_area_pointcloud_5000(rotate_pointcloud)


        cloud_point_data = np.asarray(test_pointcloud.points)
        x_boundary = (np.min(cloud_point_data[:, 0]), np.max(cloud_point_data[:, 0]))
        y_boundary = (np.min(cloud_point_data[:, 1]), np.max(cloud_point_data[:, 1]))
        # print x_boundary, y_boundary

        # np_data = np.asarray(test_pointcloud.points)
        # print np_data[:,2]
        # print np.mean(np_data[:, 2])
        # np.savetxt('./empty_dish.txt', np_data)
        # test_coo = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=test_pointcloud.points[200])
        # o3d.visualization.draw_geometries([test_pointcloud, yuandian])

        check_push_pointcloud = test_pointcloud.voxel_down_sample(voxel_size=0.01)

        if pre_need_push == True:
            push_index = self.judge_need_push(test_pointcloud, x_boundary, y_boundary, None)
        else:
            if self.can_judge == True:
                push_index = self.judge_need_push(test_pointcloud, x_boundary, y_boundary, 0.025)
            else:
                push_index = None

        # push_index = None
        if push_index != None:
            print 'initial push' + " " + str(push_index)
            action_name = 'initial push'
            waypoints = push_index
            orientation_waypoints = None

            self.monitor_push.append(waypoints)
            self.can_judge = False

            if len(self.monitor_push) != 1:
                if waypoints == self.monitor_push[0] and len(self.monitor_push) == 2:
                    # self.play_add_sound()
                    print '俩次相同位置，需要补菜'
                    waypoints = 3
                    # del self.monitor_push[:]
                    # signal_stop_receive.send('stop')
                    # rospy.sleep(0.5)
                    # return False, False

                elif len(self.monitor_push) > 2:
                    self.play_add_sound()
                    print '推3次，需要补菜'
                    del self.monitor_push[:]
                    signal_stop_receive.send('stop')
                    rospy.sleep(0.5)
                    return False, False
            # del self.monitor_four_push[:]
            if self.is_dig == True:
                is_completed = metal_push_trajectory_plan_test.push_action_trajectory_plan(action_name,
                                                                                                waypoints,
                                                                                                orientation_waypoints,
                                                                                                self.ur10_arm_spoon,
                                                                                                self.ur10_spoon_end_effector_link)
                return False, False
        else:
            # down_test_pointcloud = test_pointcloud.voxel_down_sample(voxel_size=0.0085)
            # print len(down_test_pointcloud.points)
            # o3d.visualization.draw_geometries([down_test_pointcloud, yuandian])
            points = np.asarray(test_pointcloud.points, dtype=float)
            points_color = np.asarray(test_pointcloud.colors, dtype=float)
            choice = np.random.choice(len(points), 2048, replace=True)
            xyz_points = points[choice, :]
            xyz_points_color = points_color[choice, :]

            cal_volum_points = np.asarray(cal_volum_pointcloud.points, dtype=float)
            cal_volum_points_color = np.asarray(cal_volum_pointcloud.colors, dtype=float)

            input_model_data = xyz_points

            npoints_pointcloud = o3d.geometry.PointCloud()
            npoints_pointcloud.points = o3d.utility.Vector3dVector(xyz_points)
            npoints_pointcloud.colors = o3d.utility.Vector3dVector(xyz_points_color)
            # npoints_pointcloud.paint_uniform_color([0.5,0.5,0.5])

            cal_volum_npoints_pointcloud = o3d.geometry.PointCloud()
            cal_volum_npoints_pointcloud.points = o3d.utility.Vector3dVector(cal_volum_points)
            cal_volum_npoints_pointcloud.colors = o3d.utility.Vector3dVector(cal_volum_points_color)

            # 得到模型输出
            if channels == 5:
                points_density = self.add_density(npoints_pointcloud, 0.008)
                model_result = get_model_output.get_result(points_density)
            elif channels == 4:
                points_depth = self.add_depth(xyz_points)
                model_result = get_model_output.get_result(points_depth)
            else:
                model_result = get_model_output.get_result(input_model_data)

            coloring_pointcloud, dig_area_pointcloud, analyze_start_location, analyze_end_location, \
            dig_move_pass_points, action_name, volume, sound_index, real_ave_depth = self.analyze_model_reuslt(npoints_pointcloud,
                                                                                                               model_result,
                                                                                                               cal_volum_npoints_pointcloud,
                                                                                                               x_boundary,
                                                                                                               current_dish_name)


            if dig_area_pointcloud is not None and action_name != 'corner push' and volume > 1500:
                self.can_judge = True
                del self.monitor_push[:]
                start_location, end_location = self.check_point_safely(x_boundary, y_boundary, analyze_start_location,
                                                                       analyze_end_location)

                # print start_location, end_location
                # volume = self.compute_cloud_volume(dig_area_pointcloud)
                print "总体积:" + str(volume)

                # 存储点云
                if self.is_save_cloud == True:
                    self.save_pointcloud(rotate_pointcloud)

                real_coloring_pointcloud = self.pcd_rot(coloring_pointcloud, 0, -camera_angele, 0)
                real_dig_area_pointcloud = self.pcd_rot(dig_area_pointcloud, 0, -camera_angele, 0)
                real_start_location = self.rotate_by_axis_y(start_location, -camera_angele)
                real_end_location = self.rotate_by_axis_y(end_location, -camera_angele)
                # print real_start_location
                # test_coo = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=real_start_location)
                # o3d.visualization.draw_geometries([real_coloring_pointcloud, yuandian, test_coo])

                dig_type = self.judge_dig_action(dig_area_pointcloud, 0.04, 0.7)
                print 'action_name: ' + action_name
                print 'dig_type: ' + dig_type

                if action_name == 'dig dish':
                    waypoints, orientation_waypoints = self.return_waypoints(dig_type, start_location, end_location,
                                                                             dig_move_pass_points, volume, cal_volum_pointcloud, x_boundary,
                                                                             dig_area_pointcloud, current_dish_name)
                    # print waypoints[-1]
                    # print orientation_waypoints

                    if self.is_dig == True:
                        is_completed, move_dish_plan, is_need_push = metal_push_trajectory_plan_test.push_action_trajectory_plan(
                            action_name, waypoints, orientation_waypoints, self.ur10_arm_spoon, self.ur10_spoon_end_effector_link)

                        if is_completed == True and move_dish_plan != None:
                            signal_ur10_send.send("get to pull loc")
                            rospy.sleep(0.1)
                            self.ur10_arm_spoon.execute(move_dish_plan)

                            if signal_ur3_receive.recv() == "bowl is ready":
                                # self.ur10_arm_spoon.set_joint_value_target(UR10_pull)
                                # self.ur10_arm_spoon.go()
                                # signal_ur10_send.send("pull dish finish")
                                signal_ur10_send.send("spoon is ready")
                                self.ur10_arm_spoon.execute(self.ur10_pull_dish_plan)
                                signal_ur10_send.send("pull dish finish")

                                rospy.sleep(0.1)

                                # print "ur10 go home"
                                # self.ur10_arm_spoon.execute(self.ur10_pull_to_initial_plan)

                                self.ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
                                ur10_pull_to_start_plan = self.ur10_arm_spoon.plan()

                                if len(ur10_pull_to_start_plan.joint_trajectory.points) == 0:
                                    print "读轨迹回初始位置"
                                    self.ur10_arm_spoon.set_joint_value_target(Variable.ur10_pull_finish)
                                    self.ur10_arm_spoon.go()
                                    ur10_pull_to_start_plan = self.ur10_pull_to_initial_plan
                                self.ur10_arm_spoon.execute(ur10_pull_to_start_plan)


                                # self.ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
                                # self.ur10_arm_spoon.go()

                                return True, is_need_push

                        else:
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
                self.play_add_sound()
                signal_stop_receive.send('stop')

                rospy.sleep(0.5)
                return False, False

            else:
                waypoints = self.judge_need_push(check_push_pointcloud, x_boundary, y_boundary, None)
                self.monitor_push.append(waypoints)
                self.can_judge = False

                if len(self.monitor_push) != 1:
                    if waypoints == self.monitor_push[0] and len(self.monitor_push) == 2:
                        # self.play_add_sound()

                        print '俩次相同位置，需要补菜'
                        # del self.monitor_push[:]
                        #
                        # signal_stop_receive.send('stop')
                        #
                        # rospy.sleep(0.5)
                        #
                        # return False, False
                        waypoints = 3

                    elif len(self.monitor_push) > 2:
                        self.play_add_sound()

                        print '推3次，需要补菜'
                        del self.monitor_push[:]

                        signal_stop_receive.send('stop')

                        rospy.sleep(0.5)

                        return False, False

                orientation_waypoints = None
                action_name = 'initial push'

                print 'push index', waypoints

                if self.is_dig == True:
                    if waypoints != None:
                        is_completed = metal_push_trajectory_plan_test.push_action_trajectory_plan(action_name, waypoints,
                                                                                            orientation_waypoints,
                                                                                            self.ur10_arm_spoon,
                                                                                            self.ur10_spoon_end_effector_link)
                        return is_completed, False
                    else:
                        return False, False

                # if self.test == 'True':
                #     push_start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1,
                #                                                                         origin=waypoints[0])
                #
                #     push_middle_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1,
                #                                                                          origin=waypoints[1])
                #
                #     push_end_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=waypoints[2])
                #
                #     real_npoints_pointcloud = self.pcd_rot(npoints_pointcloud, 0, -camera_angele, 0)
                #
                #     o3d.visualization.draw_geometries([real_npoints_pointcloud, yuandian, push_start_coor,
                #                                        push_middle_coor, push_end_coor])
                #     return False


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
        real_push_start_location = self.rotate_by_axis_y(push_start_point, -camera_angele)
        waypoints.append(real_push_start_location)
        real_start_push_point = self.rotate_by_axis_y(end_point, -camera_angele)
        orientation_waypoints.append(real_start_push_point)


        push_middle_point = order_move_pass_points[int(num/5)]
        deal_push_middle_point = [push_middle_point[0], push_middle_point[1], empty_depth-0.06]
        real_push_middle_location = self.rotate_by_axis_y(deal_push_middle_point, -camera_angele)
        waypoints.append(real_push_middle_location)

        push_end_point = [start_point[0], start_point[1], empty_depth - 0.05]
        real_push_end_location = self.rotate_by_axis_y(push_end_point, -camera_angele)
        waypoints.append(real_push_end_location)
        real_end_push_point = self.rotate_by_axis_y(start_point, -camera_angele)
        orientation_waypoints.append([real_end_push_point[0], real_end_push_point[1], real_start_push_point[2]])

        return waypoints, orientation_waypoints

    def return_waypoints(self, dig_type, start_point, end_point, move_pass_points, volum, cal_volum_pointcloud, x_boundary, dig_area_pointcloud, current_dish_name):
        np_move_pass_points = np.asarray(move_pass_points)
        if end_point[1] - start_point[1] > 0:
            order_move_pass_points = np_move_pass_points[np_move_pass_points[:, 1].argsort()]
        else:
            order_move_pass_points = np_move_pass_points[np.argsort(-np_move_pass_points[:, 1]), :]

        waypoints = []
        orientation_waypoints = []
        num = len(order_move_pass_points)

        if dig_type == 'full rotate':
            base_height = (start_point[2]-real_max_depth)-(1-(empty_depth-start_point[2])/7)*0.02
            print 'start point z', base_height
            deal_start_point = [start_point[0], start_point[1], base_height]
            real_start_location = self.rotate_by_axis_y(deal_start_point, -camera_angele)
            no_deal_real_start_location = self.rotate_by_axis_y(start_point, -camera_angele)
            waypoints.append(real_start_location)
            orientation_waypoints.append(no_deal_real_start_location)

            middle_point = order_move_pass_points[int(num / 2)]
            # 0.599 0.6 0.535
            # 0.529 0.61 0.55

            if middle_point[2] > max_depth:
                deal_middle_point = [middle_point[0], middle_point[1], max_depth - 0.01]
            else:
                deal_middle_point = [middle_point[0], middle_point[1], middle_point[2]]

            real_middle_location = self.rotate_by_axis_y(deal_middle_point, -camera_angele)
            waypoints.append(real_middle_location)

            orientation_middle_location = self.rotate_by_axis_y(
                (middle_point[0], middle_point[1], start_point[2] + 0.01), -camera_angele)
            orientation_waypoints.append(orientation_middle_location)

            end_point_z = self.end_point_z_judge(start_point, end_point, cal_volum_pointcloud, volum, x_boundary,
                                                 dig_area_pointcloud, current_dish_name)
            deal_end_point = [end_point[0], end_point[1], end_point_z]
            real_end_location = self.rotate_by_axis_y(deal_end_point, -camera_angele)
            # no_deal_real_end_location = self.rotate_by_axis_y(deal_end_point, -self.camera_angele)
            waypoints.append(real_end_location)

            orientation_end_location = self.rotate_by_axis_y((end_point[0], end_point[1], start_point[2]),
                                                             -camera_angele)
            orientation_last_location = self.rotate_by_axis_y((end_point[0], end_point[1], 0.5), -camera_angele)
            orientation_waypoints.append(orientation_end_location)
            orientation_waypoints.append(orientation_last_location)


        elif dig_type == 'push rotate':
            base_height = (start_point[2]-real_max_depth)-(1-(empty_depth-start_point[2])/7)*0.02
            print 'start point z', base_height
            deal_start_point = [start_point[0], start_point[1], base_height]

            real_start_location = self.rotate_by_axis_y(deal_start_point, -camera_angele)
            no_deal_real_start_location = self.rotate_by_axis_y(start_point, -camera_angele)
            waypoints.append(real_start_location)
            orientation_waypoints.append(no_deal_real_start_location)

            middle_point = order_move_pass_points[int(num / 2)]
            # 0.599 0.6 0.535
            # 0.529 0.61 0.55
            print middle_point[2]
            if middle_point[2] > max_depth:
                deal_middle_point = [middle_point[0], middle_point[1], max_depth - 0.01]

            else:
                deal_middle_point = middle_point
            real_middle_location = self.rotate_by_axis_y(deal_middle_point, -camera_angele)
            waypoints.append(real_middle_location)

            orientation_middle_location = self.rotate_by_axis_y(
                (middle_point[0], middle_point[1], start_point[2] + 0.01),
                -camera_angele)
            orientation_waypoints.append(orientation_middle_location)

            end_point_z = self.end_point_z_judge(start_point, end_point, cal_volum_pointcloud, volum, x_boundary,
                                                 dig_area_pointcloud, current_dish_name)
            deal_end_point = [end_point[0], end_point[1], end_point_z]
            real_end_location = self.rotate_by_axis_y(deal_end_point, -camera_angele)
            # no_deal_real_end_location = self.rotate_by_axis_y(deal_end_point, -self.camera_angele)
            waypoints.append(real_end_location)

            orientation_end_location = self.rotate_by_axis_y((end_point[0], end_point[1], start_point[2]), -camera_angele)
            orientation_last_location = self.rotate_by_axis_y((end_point[0], end_point[1], 0.5), -camera_angele)
            orientation_waypoints.append(orientation_end_location)
            orientation_waypoints.append(orientation_last_location)

        return waypoints, orientation_waypoints

    def end_point_z_judge(self, start_point, end_point, cal_volum_pointcloud, volum, x_boundary, dig_area_pointcloud, current_dish_name):
        square_points = []
        square_pointcloud = o3d.geometry.PointCloud()
        len_points = len(cal_volum_pointcloud.points)

        for i in range(len_points):
            if abs(cal_volum_pointcloud.points[i][0] - end_point[0]) <= 0.035 and abs(
                    cal_volum_pointcloud.points[i][1] - end_point[1]) <= 0.035:
                square_points.append(cal_volum_pointcloud.points[i])

        square_pointcloud.points = o3d.utility.Vector3dVector(square_points)
        # o3d.visualization.draw_geometries([square_pointcloud])

        total_depth = 0
        for i in range(len(square_pointcloud.points)):
            total_depth = (empty_depth - square_pointcloud.points[i][2]) + total_depth

        ave_depth = total_depth / len(square_pointcloud.points)
        print '终点平均深度', ave_depth

        dig_total_depth = 0
        for i in range(len(dig_area_pointcloud.points)):
            dig_total_depth = (empty_depth - dig_area_pointcloud.points[i][2]) + dig_total_depth

        dig_ave_depth = dig_total_depth / len(dig_area_pointcloud.points)
        print '打菜区域平均深度', dig_ave_depth

        allowMax_height = end_point[2] - real_max_depth

        # if dig_ave_depth > ave_depth or ave_depth > 0.045:
        # if ave_depth > 0.45 and dig_ave_depth > 1:
        #     end_point_z = allowMax_height
        #     print 'end_point_z=allowmax'
        #
        # elif dig_ave_depth > ave_depth:
        #     end_point_z = allowMax_height + (max_depth - allowMax_height) * (1 - ave_depth / 0.07)
        #     print 'end_point_z=1111'
        if dig_ave_depth > ave_depth or (ave_depth > 0.045 and dig_ave_depth > 0.035):
            if current_dish_name == 'ChaoPuGua' or current_dish_name == 'RouPiHuangDou' or current_dish_name == 'ZhaTuDou':
                end_point_z = allowMax_height
                print 'allowmax'
            else:
                print 'allowMax+0.01'
                end_point_z = allowMax_height + 0.01
        else:
            print '公式'
            end_point_z = (max_depth - allowMax_height) * (ave_depth / dig_ave_depth - ave_depth / 0.035) + allowMax_height

        if end_point_z > max_depth:
            print 'max'
            end_point_z = max_depth

        print 'end_point_z', end_point_z
        return end_point_z

    def judge_need_push(self, cloud, x_boundary, y_boundary, tolerance):
        total = 0

        for i in range(len(cloud.points)):
            total = total + empty_depth - cloud.points[i][2]
        aver_total = total * 1.0 / len(cloud.points)

        print 'aver_total', aver_total
        if aver_total > 0.04 and tolerance != None:
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
                    four_corner_points_depth[0] += (empty_depth - cloud.points[i][2])
                    four_corner_num[0] += 1

                elif cloud.points[i][0] > x_three[2] and cloud.points[i][1] > y_three[2]:
                    four_corner_points_depth[1] += (empty_depth - cloud.points[i][2])
                    four_corner_num[1] += 1

                elif cloud.points[i][0] < x_three[1] and cloud.points[i][1] > y_three[2]:
                    four_corner_points_depth[2] += (empty_depth - cloud.points[i][2])
                    four_corner_num[2] += 1

                elif cloud.points[i][0] < x_three[1] and cloud.points[i][1] < y_three[1]:
                    four_corner_points_depth[3] += (empty_depth - cloud.points[i][2])
                    four_corner_num[3] += 1

                elif x_three[1] < cloud.points[i][0] < x_three[2] and y_three[1] < cloud.points[i][1] < y_three[2]:
                    center_points_depth += (empty_depth - cloud.points[i][2])
                    center_num += 1

            center_ave_depth = center_points_depth * 1.0 / center_num

            print 'center_ave_depth', center_ave_depth

            satisfied_target_area = []
            satisfied_target_area_index = []
            for i in range(4):
                aver_depth = four_corner_points_depth[i] * 1.0 / four_corner_num[i]

                print 'aver_depth', aver_depth

                # 分布不均匀
                if tolerance != None:
                    if aver_depth > 0.04:
                        if i == 0 or i == 1 or i == 3:
                            aver_depth = aver_depth + 0.01
                        elif i == 2:
                            aver_depth = aver_depth - 0.015
                    else:
                        if i == 2:
                            aver_depth -= 0.005
                else:
                    if i == 0 or i == 1 or i == 3:
                        aver_depth = aver_depth + 0.01
                    elif i == 2:
                        aver_depth = aver_depth - 0.015

                if tolerance == None:
                    satisfied_target_area.append(aver_depth)
                else:
                    if aver_depth - center_ave_depth > tolerance:
                        satisfied_target_area_index.append(i)

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

        return max_index

    def add_bowl_to_device_up(self, ur3_arm_ag95, AG95_client):
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
        rospy.sleep(0.2)
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
        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()

    def play_add_sound(self):
        playsound('/home/robot/sound/add.mp3')

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

    def generate_bowl_loc(self, bowl_num):
        bowl_loc = []
        bowl_up_loc = []
        if bowl_num == 1:
            bowl_loc = one_bowl__plate_loc
            bowl_up_loc = one_bowl_up_loc
        if bowl_num % 2 == 0:
            ite = bowl_num / 2
            for i in range(ite):
                bowl_loc.append(two_bowl_plate_loc[0])
                bowl_loc.append(two_bowl_plate_loc[1])
                bowl_up_loc.append(two_bowl_up_loc[0])
                bowl_up_loc.append(two_bowl_up_loc[1])
        elif bowl_num % 2 == 1:
            ite = bowl_num / 2
            for i in range(ite):
                bowl_loc.append(two_bowl_plate_loc[0])
                bowl_loc.append(two_bowl_plate_loc[1])
                bowl_up_loc.append(two_bowl_up_loc[0])
                bowl_up_loc.append(two_bowl_up_loc[1])
            bowl_loc.append(one_bowl__plate_loc[0])
            bowl_up_loc.append((one_bowl_up_loc[0]))
        return bowl_loc, bowl_up_loc

    def compute_cloud_volume(self, cloud):
        # real_cloud = self.pcd_rot(cloud, 0, -camera_angele, 0)
        density = 31 * 46 *1.0 / 2500
        real_depth = 0
        for i in range(len(cloud.points)):
            depth = (-cloud.points[i][2] + empty_depth) * 100
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
            # print middle_area_pointcloud.points[i][2]
            if (empty_depth-middle_area_pointcloud.points[i][2] > bottom_threshold):
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
        # print 'cal volum pointcloud下采样后点数', len(dish_area_pointcloud.points)
        return dish_area_pointcloud

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

        if start_center_point[1] - end_center_point[1] < -0.03 and start_center_point[0] - end_center_point[0] > 0.01:
            action_name = 'dig dish'
            require_volum = volume_goal[current_dish_name]
        elif start_center_point[1] - end_center_point[1] > 0.01 and start_center_point[0] - end_center_point[0] > 0.01:
            action_name = 'corner push1'
            require_volum = 0
        elif abs(start_center_point[0] - end_center_point[0]) < 0.01:
            action_name = 'corner push2'
            require_volum = 0
        else:
            action_name = 'corner push3'
            require_volum = 0

        print 'action_name', action_name
        if len(start_pointcloud.points) > 50 and len(end_pointcloud.points) > 50 and action_name == 'dig dish':
            middle_area_pointcloud, dig_move_pass_points, start_location, end_location, volume, real_ave_depth = self.get_required_volum_area(start_center_point,
                                                                                                                              end_center_point,
                                                                                                                              0.1,
                                                                                                                              require_volum,
                                                                                                                              cal_volum_npoints_pointcloud,
                                                                                                                              x_boundary,
                                                                                                                              cloud)

            # middle_area_pointcloud_tree = o3d.geometry.KDTreeFlann(middle_area_pointcloud)
            # [_, start_idx, _] = middle_area_pointcloud_tree.search_knn_vector_3d(end_location, int(len(middle_area_pointcloud.points)/5))
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


    def get_required_volum_area(self, point_1, point_2, area_width, volum, cal_volum_npoints_pointcloud, x_boundary, test_cloud):
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
        real_ave_depth = empty_depth - ave_depth

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

        # if total_dis < 0.1:
        #     index = 0.1 / every_dis
        #     print "修改", index
        #     total_dis = 0.1

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

        up_half_x = x_boundary[0] + (x_boundary[1] - x_boundary[0]) / 2
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
                left_up_volum = left_up_volum + (empty_depth - complete_cloud.points[i][2]) * 100
            elif complete_cloud.points[i][0] > cloud_center_loc[0] and complete_cloud.points[i][1] > cloud_center_loc[1]:
                right_up_volum = right_up_volum + (empty_depth - complete_cloud.points[i][2]) * 100
            elif complete_cloud.points[i][0] < cloud_center_loc[0] and complete_cloud.points[i][1] > cloud_center_loc[1]:
                right_down_volum = right_down_volum + (empty_depth - complete_cloud.points[i][2]) * 100
            elif complete_cloud.points[i][0] < cloud_center_loc[0] and complete_cloud.points[i][1] < cloud_center_loc[1]:
                left_down_volum = left_down_volum + (empty_depth - complete_cloud.points[i][2]) * 100

            if four_corner_loc_copy[0][0]<complete_cloud.points[i][0] < four_corner_loc[0][0] and four_corner_loc[0][1]<complete_cloud.points[i][1] < four_corner_loc_copy[0][1]:
                left_up_points.append(complete_cloud.points[i])
            if four_corner_loc_copy[1][0]<complete_cloud.points[i][0] < four_corner_loc[1][0] and four_corner_loc_copy[1][1]<complete_cloud.points[i][1] < four_corner_loc[1][1]:
                right_up_points.append(complete_cloud.points[i])
            if four_corner_loc[2][0]<complete_cloud.points[i][0] < four_corner_loc_copy[2][0] and four_corner_loc_copy[2][1]<complete_cloud.points[i][1] < four_corner_loc[2][1]:
                right_down_points.append(complete_cloud.points[i])
            if four_corner_loc[3][0]<complete_cloud.points[i][0] < four_corner_loc_copy[3][0] and four_corner_loc[3][1]<complete_cloud.points[i][1] < four_corner_loc_copy[3][1]:
                left_down_points.append(complete_cloud.points[i])

        # four_area_list = [left_up_volum, right_up_volum, right_down_volum, left_down_volum]
        four_area_list = [left_up_volum, right_up_volum]

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
        # if target_area_index == 2:
        #     for i in range(len(right_down_points)):
        #         if right_down_points[i][2] < target_push_start[2]:
        #             target_push_start = right_down_points[i]
        # if target_area_index == 3:
        #     for i in range(len(left_down_points)):
        #         if left_down_points[i][2] < target_push_start[2]:
        #             target_push_start = left_down_points[i]

        waypoints = []
        deal_target_push_start = [target_push_start[0], target_push_start[1], target_push_start[2]-real_max_depth]
        real_target_push_start = self.rotate_by_axis_y(deal_target_push_start, -camera_angele)
        waypoints.append(real_target_push_start)

        yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0,0,0])
        print target_push_start

        if target_area_index == 0:
            target_push_middle_x = max(target_push_start[0], cloud_center_loc[0]) - ( abs(target_push_start[0] - cloud_center_loc[0]) * 1.0 / 5)
            target_push_middle_y = min(target_push_start[1], cloud_center_loc[1]) + (abs(target_push_start[1] - cloud_center_loc[1])*1.0/5)
        elif target_area_index == 1:
            target_push_middle_x = max(target_push_start[0], cloud_center_loc[0]) - ( abs(target_push_start[0] - cloud_center_loc[0]) * 1.0 / 5)
            target_push_middle_y = max(target_push_start[1], cloud_center_loc[1]) - ( abs(target_push_start[1] - cloud_center_loc[1]) * 1.0 / 5)
        # elif target_area_index == 2:
        #     target_push_middle_x = min(target_push_start[0], cloud_center_loc[0]) + ( abs(target_push_start[0] - cloud_center_loc[0]) * 1.0 / 5)
        #     target_push_middle_y = max(target_push_start[1], cloud_center_loc[1]) - ( abs(target_push_start[1] - cloud_center_loc[1]) * 1.0 / 5)
        # elif target_area_index == 3:
        #     target_push_middle_x = min(target_push_start[0], cloud_center_loc[0]) + ( abs(target_push_start[0] - cloud_center_loc[0]) * 1.0 / 5)
        #     target_push_middle_y = min(target_push_start[1], cloud_center_loc[1]) + ( abs(target_push_start[1] - cloud_center_loc[1]) * 1.0 / 5)

        target_push_middle = [target_push_middle_x, target_push_middle_y, empty_depth-0.05]
        real_target_push_middle = self.rotate_by_axis_y(target_push_middle, -camera_angele)
        waypoints.append(real_target_push_middle)

        deal_target_push_end = [cloud_center_loc[0], cloud_center_loc[1], empty_depth - 0.045]
        real_target_push_end = self.rotate_by_axis_y(deal_target_push_end, -camera_angele)
        waypoints.append(real_target_push_end)

        rotate_message = []
        theta = math.atan2(abs(target_push_start[0] - cloud_center_loc[0]), abs(target_push_start[1] - cloud_center_loc[1]))
        if target_area_index == 0 or target_area_index == 2:
            rotate_message = [[math.pi/2, -math.pi/2, -math.pi/2-theta, -math.pi/2], 'zyxz']
        elif target_area_index == 1 or target_area_index == 3:
            rotate_message = [[math.pi / 2, -math.pi / 2, math.pi / 2 + theta, -math.pi / 2], 'zyxz']
        # elif target_area_index == 2:
        #     rotate_message = [[math.pi / 2, -math.pi / 2, math.pi / 2 + theta, -math.pi / 2], 'zyxz']
        # elif target_area_index == 3:
        #     rotate_message = [[math.pi / 2, -math.pi / 2, -theta, -math.pi / 2], 'zyxz']

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
        density = 31 * 46 * 1.0 / len_points  #每个点所占面积
        for i in range(len(cloud.points)):
            if low <= cloud.points[i][0] <= high:
                total_depth = (empty_depth - cloud.points[i][2]) * 100 + total_depth

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

    def add_bowl(self):
        bowl_file = '/home/robot/catkin_ws/src/bowl_urdf/meshes/bowl.STL'
        bowl_loc = [0, 0, 0]
        bowl_rot = tf.transformations.quaternion_from_euler(3.14, 0, 0)

        bowl_pose = geometry_msgs.msg.PoseStamped()
        bowl_pose.header.frame_id = 'grip_tip_link'
        bowl_pose.header.stamp = rospy.Time.now()
        bowl_pose.pose.position.x = bowl_loc[0]
        bowl_pose.pose.position.y = bowl_loc[1]
        bowl_pose.pose.position.z = bowl_loc[2]
        bowl_pose.pose.orientation.x = bowl_rot[0]
        bowl_pose.pose.orientation.y = bowl_rot[1]
        bowl_pose.pose.orientation.z = bowl_rot[2]
        bowl_pose.pose.orientation.w = bowl_rot[3]

        self.ur3_scene.attachMesh("bowl", bowl_pose.pose, bowl_file, "grip_tip_link")
        # self.ur3_scene.addMesh("bowl", bowl_pose.pose, bowl_file, (1,1,1))

        # print 1
    def delete_bowl(self):
        self.ur3_scene.removeAttachedObject("bowl")

    def add_desk_belt(self):
        desk_file = '/home/robot/catkin_ws/src/ur3ur10car/desk/meshes/desk.STL'
        desk_loc = [-0.77, -0.815, 0.705]
        desk_rot = tf.transformations.quaternion_from_euler(3.14,0,-1.57)

        desk_pose = geometry_msgs.msg.PoseStamped()
        desk_pose.header.frame_id = 'body_link'
        desk_pose.header.stamp = rospy.Time.now()
        desk_pose.pose.position.x = desk_loc[0]
        desk_pose.pose.position.y = desk_loc[1]
        desk_pose.pose.position.z = desk_loc[2]
        desk_pose.pose.orientation.x = desk_rot[0]
        desk_pose.pose.orientation.y = desk_rot[1]
        desk_pose.pose.orientation.z = desk_rot[2]
        desk_pose.pose.orientation.w = desk_rot[3]

        self.ur3_scene.addMesh("desk", desk_pose.pose, desk_file, (1,1,1))
        self.ur3_scene.setColor("desk", 1,0,0,0.9)

        belt_file = '/home/robot/catkin_ws/src/ur3ur10car/conveyor_belt/meshes/conveyor_belt.STL'
        belt_loc = [-0.77, -0.8, 0.65]
        belt_rot = tf.transformations.quaternion_from_euler(3.14,0,0)

        belt_pose = geometry_msgs.msg.PoseStamped()
        belt_pose.header.frame_id = 'odom'
        belt_pose.header.stamp = rospy.Time.now()
        belt_pose.pose.position.x = belt_loc[0]
        belt_pose.pose.position.y = belt_loc[1]
        belt_pose.pose.position.z = belt_loc[2]
        belt_pose.pose.orientation.x = belt_rot[0]
        belt_pose.pose.orientation.y = belt_rot[1]
        belt_pose.pose.orientation.z = belt_rot[2]
        belt_pose.pose.orientation.w = belt_rot[3]

        self.ur3_scene.addMesh("belt", belt_pose.pose, belt_file, (1, 1, 1))


if __name__ == '__main__':

    try:
        model_effect_test()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass



    # def open_pipeline(self):
    #     align = rs.align(rs.stream.color)
    #
    #     config = rs.config()
    #     config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    #     config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
    #     pipeline = rs.pipeline()
    #     profile = pipeline.start(config)
    #
    #     # get camera intrinsics
    #     intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    #     # print(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
    #     pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(intr.width, intr.height, intr.fx, intr.fy,
    #                                                                  intr.ppx, intr.ppy)
    #     # print(type(pinhole_camera_intrinsic))
    #     return pipeline, pinhole_camera_intrinsic
    #
    # def get_xyz_points(self, pipeline, pinhole_camera_intrinsic):
    #     frame = pipeline.wait_for_frames()
    #     color_frame = frame.get_color_frame()
    #     depth_frame = frame.get_depth_frame()
    #
    #     color_image = np.asanyarray(color_frame.get_data())
    #     depth_image = np.asanyarray(depth_frame.get_data())
    #
    #     depth = o3d.geometry.Image(depth_image)
    #     color = o3d.geometry.Image(color_image)
    #
    #     rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False)
    #     original_pointcloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)
    #
    #     rotate_pointcloud = self.pcd_rot(original_pointcloud, 0, 0.599, 0)
    #     yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3, origin=[0, 0, 0])
    #     test_pointcloud = self.cut_safe_area_pointcloud(rotate_pointcloud)
    #
    #     points = np.asarray(test_pointcloud.points, dtype=float)
    #     # change 2500 points
    #     choice = np.random.choice(len(points), 2500, replace=True)
    #     xyz_points = points[choice, :]
    #
    #     npoints_pointcloud = o3d.geometry.PointCloud()
    #     npoints_pointcloud.points = o3d.utility.Vector3dVector(xyz_points)
    #     npoints_pointcloud.paint_uniform_color([0.5, 0.5, 0.5])
    #     # o3d.visualization.draw_geometries([npoints_pointcloud])
    #
    #     # 得到模型输出
    #     if channels == 5:
    #         points_density = self.add_density(npoints_pointcloud, 0.008)
    #         model_result = get_model_output.get_result(points_density)
    #     elif channels == 4:
    #         points_depth = self.add_depth(xyz_points)
    #         model_result = get_model_output.get_result(points_depth)
    #     elif channels == 3:
    #         model_result = get_model_output.get_result(xyz_points)
    #     print model_result
    #
    #     coloring_pointcloud, dig_area_pointcloud, start_location, end_location, dig_move_pass_points = self.analyze_model_reuslt(
    #         npoints_pointcloud, model_result)
    #
    #     # print start_location, end_location
    #
    #     if dig_area_pointcloud is not None:
    #         # 可视化结果
    #         # npoints_pointcloud_color = self.select_points(npoints_pointcloud, model_result)
    #
    #         # x, y = self.get_rotate_angle(start_location, end_location)
    #         # p = start_coor.get_rotation_matrix_from_xyz((x,math.pi-y,0))
    #         real_coloring_pointcloud = self.pcd_rot(coloring_pointcloud, 0, -0.599, 0)
    #         real_start_location = self.rotate_by_axis_y(start_location, -0.599)
    #         real_end_location = self.rotate_by_axis_y(end_location, -0.599)
    #
    #         # start_rot_message = tf.TransformBroadcaster()
    #         # start_rot_message.sendTransform((real_start_location[0], real_start_location[1], real_start_location[2]),
    #         #                                 quaa,
    #         #                                 rospy.Time.now(),
    #         #                                 "/start_loca", "/camera2_depth_optical_frame")
    #
    #         action_name = self.judge_dig_action(dig_area_pointcloud, 0.04, 0.6)
    #         print action_name
    #         if action_name == 'full rotation':
    #             waypoints = self.return_waypoints(action_name, start_location, end_location, dig_move_pass_points)
    #             interpolation_points = dig_dish_trajectory_plan.action_trajectory_plan(action_name, waypoints)
    #
    #             np_interpolation_points = np.asarray(interpolation_points)
    #
    #             min_loc_index = np.argsort(np_interpolation_points[:, 2])[99]
    #
    #             real_x, real_y = self.get_rotate_angle_to_end(interpolation_points[min_loc_index],
    #                                                           interpolation_points[min_loc_index + 30])
    #             # real_x, real_y = self.get_rotate_angle_to_end([0,0,1], [0,0,0])
    #             # print interpolation_points[min_loc_index], interpolation_points[min_loc_index + 30]
    #             # quaa = self.compute_rotate_matrix(real_x,0,0,'xyz')
    #             start_coor = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1,
    #                                                                            origin=interpolation_points[
    #                                                                                min_loc_index])
    #             p = start_coor.get_rotation_matrix_from_xyz((real_x, real_y, 0))
    #             start_coor.rotate(p)
    #
    #         o3d.visualization.draw_geometries([real_coloring_pointcloud, yuandian, start_coor])
    #     else:
    #         self.get_xyz_points(pipeline, pinhole_camera_intrinsic)