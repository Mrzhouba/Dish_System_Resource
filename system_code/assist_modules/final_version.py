#!/usr/bin/env python
#coding=UTF-8
import tf
import rospy
import moveit_commander
import roslib; roslib.load_manifest('ur_driver')
import open3d as o3d
import paho.mqtt.client as mqtt
import copy
import json
import actionlib
import sys
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/assist_modules')
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/pointnet2_pytorch/utils')
import multiprocessing
import threading
import resource
import psutil
import serial, binascii
import argparse
import Variable
from copy import deepcopy
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from sensor_msgs.msg import JointState,Image,CameraInfo, PointCloud2
from open3d_ros_helper import open3d_ros_helper as orh
from trajectory_msgs.msg import *
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


from grippers_control import grippers_control
from scan_dish import scan_dish
from serial_drop_bowl import drop_bowl
from acquire_orders import acquire_orders_and_feedback
from final_judge_dish_location import final_judge_dish_location


grippers_control = grippers_control()
scan_dish = scan_dish()
drop_bowl = drop_bowl()
acquire_orders_and_feedback = acquire_orders_and_feedback()
final_judge_dish_location = final_judge_dish_location()

HOST = Variable.HOST
PORT = Variable.PORT
car_mqtt_client = mqtt.Client("control_car")
bowl_drop_mqtt_client = mqtt.Client("control_bowl_drop")
convey_mqtt_client = mqtt.Client("control_convey")

#拍摄准备点
UR10_Q1_to_left = Variable.UR10_Q1_to_left

#准备点
UR3_Q1 = Variable.UR3_Q1
#取碗点
# UR3_get_bowl_loction = [-1.1843646208392542, -1.4627745787249964, -1.9370248953448694, 0.28942692279815674, 1.2918339967727661, 4.7266621589660645]
#过渡点
# UR3_transition_location = [-0.5971816221820276, -1.537464443837301, -1.7267301718341272, 0.17633211612701416, 1.6091862916946411, 4.714641571044922]
#放碗点
#old dual-arm-move-together
# UR3_lay_location = [0.7378380298614502, -1.7818334738360804, -1.903487507496969, 0.5219758749008179, 1.3935306072235107, 4.712612152099609]

UR3_lay_location = Variable.UR3_lay_location
#离开点
# UR3_lay_up_location = [0.21412739157676697, -1.7258804480182093, -2.0050109068499964, 0.6107040643692017, 1.3218027353286743, 4.714305400848389]

# one_bowl_loc = [[0.6785475611686707, -2.143420998250143, -1.6524904409991663, 0.632731556892395, 1.4522337913513184, 4.710690498352051]]
# two_bowl_loc = [[0.5609722137451172, -2.3851164023028772, -1.1958144346820276, 0.4179569482803345, 1.5704615116119385, 4.708253383636475],
#                 [0.854677140712738, -1.9196131865130823, -2.0202696959124964, 0.7756901979446411, 1.2757338285446167, 4.7146172523498535]]

# with plate
one_bowl_loc = Variable.one_bowl_plate_loc
two_bowl_loc = Variable.two_bowl_plate_loc

one_bowl_up_loc = Variable.one_bowl_up_loc
two_bowl_up_loc = Variable.two_bowl_up_loc

parser = argparse.ArgumentParser(description='point cloud paraamenters')
parser.add_argument('--model_name', type=str, default=Variable.model_name)
parser.add_argument('--weight_path', type=str, default=Variable.weight_path)
parser.add_argument('--channels', type=int, default=Variable.channels)
parser.add_argument('--nclasses', type=int, default=Variable.nclasses)

ur10_max_acceleration = Variable.ur10_max_acceleration
ur10_max_velocity = Variable.ur10_max_velocity

ur3_max_acceleration = Variable.ur3_max_acceleration
ur3_max_velocity = Variable.ur3_max_velocity

pause_usb_name = Variable.pause_usb_name
order_usb_name = Variable.order_usb_name


point_network_args = parser.parse_args()

class final_ur3ur10_alone_work():
    def __init__(self):
        dish_location_relative_odom = {}
        fn = sys.stdin.fileno()
        self.current_car_location = 0
        self.judge_push = False
        self.dish_box_depth_in_camera = 0.6
        self.plate_signal = True

        signal_ur3_send, signal_ur3_receive = multiprocessing.Pipe(True)
        signal_ur10_send, signal_ur10_receive = multiprocessing.Pipe(True)
        signal_dig_start_send, signal_dig_start_receive = multiprocessing.Pipe(True)

        signal_3_car_send, signal_3_car_receive = multiprocessing.Pipe(True)
        # signal_10_car_send, signal_10_car_receive = multiprocessing.Pipe(True)
        signal_plate_send, signal_plate_receive = multiprocessing.Pipe(True)

        move_3 = multiprocessing.Process(target=self.move_ur3_test, args=(signal_ur3_send, signal_ur10_receive, signal_3_car_send, signal_plate_receive))
        move_10 = multiprocessing.Process(target=self.move_ur10, args=(signal_ur3_receive, signal_ur10_send, signal_dig_start_receive, ))
        move_car = multiprocessing.Process(target=self.move_car, args=(signal_3_car_receive, signal_ur10_receive, signal_dig_start_send, ))
        monitor_plate = multiprocessing.Process(target=self.monitor_plate, args=(signal_plate_send, ))

        move_3.start()
        move_3_pid = move_3.pid
        move_10.start()
        move_10_pid = move_10.pid
        move_car.start()
        move_car_pid = move_car.pid

        monitor_plate.start()

        pause_process = multiprocessing.Process(target=self.pause_process, args=(move_3_pid, move_10_pid, move_car_pid))
        pause_process.start()

        move_3.join()
        move_10.join()
        move_car.join()
        pause_process.join()
        monitor_plate.join()

    def pause_process(self, move_3_pid, move_10_pid, move_car_pid):
        ur3_pause = psutil.Process(move_3_pid)
        ur10_pause = psutil.Process(move_10_pid)
        car_pause = psutil.Process(move_car_pid)
        portx = pause_usb_name
        bps = 9600
        timex = None
        ser = serial.Serial(portx, bps, timeout=timex)
        while 1:
            data = binascii.b2a_hex(ser.read())
            if data == "03":
                ur3_pause.suspend()
                ur10_pause.suspend()
                car_pause.suspend()
                print('打菜暂停运行')

            data = binascii.b2a_hex(ser.read())
            if data == "03":
                ur3_pause.resume()
                ur10_pause.resume()
                car_pause.resume()
                print('打菜恢复运行')

    def monitor_plate(self, signal_plate_send):
        portx = order_usb_name
        bps = 9600
        timex = None

        while 1:
            ur3_request = signal_plate_send.recv()
            if ur3_request == 'wait plate':
                ser = serial.Serial(portx, bps, timeout=timex)
                while 1:
                    data = binascii.b2a_hex(ser.read())
                    if data == '01':
                        signal_plate_send.send("plate is ready")
                        ser.close()
                        break

    def move_ur10(self, signal_ur3_receive, signal_ur10_send, signal_dig_start_receive):
        # sys.stdin = os.fdopen(fn)
        from get_model_output import get_model_output
        self.get_model_output = get_model_output(point_network_args.model_name, point_network_args.channels, point_network_args.nclasses,
                                            point_network_args.weight_path)
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("dig_dish", anonymous=True, disable_signals=True)
        reference_frame = "body_link"
        ur10_group_name = "ur10_with_gripper"
        ur10_arm_spoon = moveit_commander.MoveGroupCommander(ur10_group_name, ns="ur10")

        # 当运动规划失败后，允许重新规划
        ur10_arm_spoon.allow_replanning(False)
        ur10_arm_spoon.set_planning_time(1.0)

        # 设置目标位置所使用的参考坐标系
        ur10_arm_spoon.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        ur10_arm_spoon.set_goal_position_tolerance(0.001)
        ur10_arm_spoon.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        ur10_arm_spoon.set_max_acceleration_scaling_factor(ur10_max_acceleration)
        ur10_arm_spoon.set_max_velocity_scaling_factor(ur10_max_velocity)

        # 获取终端link的名称
        ur10_spoon_end_effector_link = ur10_arm_spoon.get_end_effector_link()

        ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
        ur10_arm_spoon.go()

        #获取菜品位置
        dish_location_relative_odom = self.read_all_dish_location_dictionary()
        args = [ur10_arm_spoon, ur10_spoon_end_effector_link, signal_ur10_send, signal_ur3_receive,
                self.get_model_output]
        while not rospy.is_shutdown():

            # self.point_sub = rospy.Subscriber("/camera2/depth/color/points", PointCloud2, final_judge_dish_location.points_callback,
            #                                   (ur10_arm_spoon, ur10_spoon_end_effector_link, signal_ur10_send, signal_ur3_receive, signal_dig_start_receive))
            # rospy.spin()
            start_signal = signal_dig_start_receive.recv()
            if start_signal == 'dig start':
                print "start"
                completed = False
                while not completed:
                    point_cloud = rospy.wait_for_message("/camera2/depth/color/points", PointCloud2, timeout=1)
                    completed = final_judge_dish_location.points_callback(point_cloud, args)
                    print completed


    def move_ur3(self, signal_ur3_send, signal_ur10_receive, signal_3_car_send, signal_plate_receive):
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
        ur3_arm_ag95.set_max_acceleration_scaling_factor(ur3_max_acceleration)
        ur3_arm_ag95.set_max_velocity_scaling_factor(ur3_max_velocity)

        # 获取终端link的名称
        ur3_end_effector_link = ur3_arm_ag95.get_end_effector_link()

        bowl_drop_mqtt_client.connect(HOST, PORT, 60)
        bowl_drop_mqtt_client.loop_start()

        convey_mqtt_client.connect(HOST, PORT, 60)
        convey_mqtt_client.loop_start()

        AG95_client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)

        ur3_get_bowl_plan = self.read_tra_message(Variable.ur3_get_bowl_plan)

        ur3_add_bowl_plan = self.read_tra_message(Variable.ur3_add_bowl_plan)
        ur3_add_bowl_to_device_plan_2, ur3_place_loc_2 = self.read_tra_message_with_place(Variable.ur3_to_device_up_plan_2)
        ur3_add_bowl_to_device_plan_3, ur3_place_loc_3 = self.read_tra_message_with_place(Variable.ur3_to_device_up_plan_3)
        ur3_add_bowl_to_device_plan_4, ur3_place_loc_4 = self.read_tra_message_with_place(Variable.ur3_to_device_up_plan_4)
        ur3_add_bowl_to_device_plan = [ur3_add_bowl_to_device_plan_2, ur3_add_bowl_to_device_plan_3, ur3_add_bowl_to_device_plan_4]
        ur3_place_bowl_to_device_loc = [ur3_place_loc_2, ur3_place_loc_3, ur3_place_loc_4]

        ur3_pull_dish_plan = self.read_tra_message(Variable.ur3_pull_dish_plan)

        ur3_get_start_plan = self.read_tra_message(Variable.ur3_get_start_plan)
        ur3_arm_ag95.execute(ur3_get_start_plan)

        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()

        order_num = 0
        no_order_time = 0

        # 接收订单
        for i in range(1000):
            foodAndNum, order_id = acquire_orders_and_feedback.acquire_orders()

            if foodAndNum != 'no order' and order_id != 0:
                order_num += 1
                dish_num = len(foodAndNum)
                bowl_num = 0
                dish_list = []

                for i in range(dish_num):
                    bowl_num += foodAndNum[foodAndNum.keys()[i]]
                    for _ in range(foodAndNum[foodAndNum.keys()[i]]):
                        dish_list.append(str(foodAndNum.keys()[i]).replace('u\'', '\''))

                ordef_list = [bowl_num, dish_list]

                #一个订单中需要碗的数量
                # ordef_list = [[5,['Candy', 'Peanut', 'Candy', 'SunflowerSeed', 'Peanut']], [3, ['Candy', 'Peanut', 'SunflowerSeed']]]
                # ordef_list = [3, ['Candy', 'SunflowerSeed', 'Peanut']]

                need_num_bowl = ordef_list[0]
                dish_list = ordef_list[1]

                if need_num_bowl == len(dish_list):
                    print "-------------------------------------------------------------------------------"
                    print "执行第{}个订单".format(order_num)
                    acquire_orders_and_feedback.set_order_running(order_id)
                    bowl_loc, bowl_up_loc = self.generate_bowl_loc(need_num_bowl)

                    all_dish_location_dictionary = self.read_all_dish_location_dictionary()

                    signal_plate_receive.send('wait plate')
                    signal_plate_ready = signal_plate_receive.recv()
                    convey_tranfer_tray_dis = all_dish_location_dictionary[dish_list[0]] - 1.5
                    drop_bowl.control_convey_belt(convey_tranfer_tray_dis, convey_mqtt_client, False)
                    current_convey_location = all_dish_location_dictionary[dish_list[0]]


                    signal_3_car_send.send("have new orders")
                    signal_3_car_send.send(need_num_bowl)

                    for i in range(need_num_bowl):

                        signal_3_car_send.send(dish_list[i])

                        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
                        ur3_arm_ag95.go()

                        # ur3_arm_ag95.set_joint_value_target(UR3_get_bowl_loction)
                        ur3_arm_ag95.execute(ur3_get_bowl_plan)

                        rospy.sleep(0.2)

                        grippers_control.AG95_gripper(100, 0, AG95_client)
                        drop_finish = drop_bowl.send_order_drop_bowl(bowl_drop_mqtt_client)

                        if drop_finish == True:

                            dish_dig_finish = signal_ur10_receive.recv()

                            if dish_dig_finish == "get to pull loc":
                                ur3_arm_ag95.set_joint_value_target(UR3_lay_location)
                                ur3_arm_ag95.go()

                                signal_ur3_send.send("bowl is ready")
                                start_dish_signal = signal_ur10_receive.recv()
                                if start_dish_signal == "start pull dish":
                                    ur3_arm_ag95.execute(ur3_pull_dish_plan)

                                    ur3_arm_ag95.set_joint_value_target(bowl_loc[i])
                                    ur3_arm_ag95.go()

                                    grippers_control.AG95_gripper(100, 55, AG95_client)

                                    rospy.sleep(0.2)
                                    ur3_arm_ag95.set_joint_value_target(bowl_up_loc[i])
                                    ur3_arm_ag95.go()

                                    if i == need_num_bowl - 1:
                                        # distance_exit = all_dish_location_dictionary[dish_list[-1]] - 0.18*need_num_bowl
                                        distance_exit = 1.5 - current_convey_location
                                        drop_bowl.control_convey_belt(distance_exit, bowl_drop_mqtt_client, False)
                                    elif i % 2 == 0 and i != need_num_bowl-1:
                                        next_distance = all_dish_location_dictionary[dish_list[i + 1]] - current_convey_location
                                        drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
                                        current_convey_location = current_convey_location + next_distance

                                    elif i % 2 == 1 and i != need_num_bowl-1:
                                        next_distance = -0.145 + all_dish_location_dictionary[dish_list[i+1]] - current_convey_location
                                        drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
                                        current_convey_location = current_convey_location + next_distance + 0.145

                    ur3_arm_ag95.set_joint_value_target(UR3_Q1)
                    ur3_arm_ag95.go()

                    acquire_orders_and_feedback.set_order_finish(order_id)
                    no_order_time = 0
                    print "----------------------------------------------------------------------"
                else:
                    print "error oders, please check!"
            # 复位信号
            else:
                print 'no order, acquire again'
                no_order_time += 1
                rospy.sleep(1)
                if no_order_time == 60:
                    no_order_time = 0
                    signal_3_car_send.send("go home")
                    break

        # signal_3_car_send.send("go home")
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)

    def move_ur3_test(self, signal_ur3_send, signal_ur10_receive, signal_3_car_send, signal_plate_receive):
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
        ur3_arm_ag95.set_max_acceleration_scaling_factor(0.1)
        ur3_arm_ag95.set_max_velocity_scaling_factor(0.1)

        # 获取终端link的名称
        ur3_end_effector_link = ur3_arm_ag95.get_end_effector_link()

        bowl_drop_mqtt_client.connect(HOST, PORT, 60)
        bowl_drop_mqtt_client.loop_start()

        convey_mqtt_client.connect(HOST, PORT, 60)
        convey_mqtt_client.loop_start()

        AG95_client = actionlib.SimpleActionClient('actuate_hand', ActuateHandAction)

        ur3_get_bowl_plan = self.read_tra_message(Variable.ur3_get_bowl_plan)

        ur3_add_bowl_plan = self.read_tra_message(Variable.ur3_add_bowl_plan)
        ur3_add_bowl_to_device_plan_2, ur3_place_loc_2 = self.read_tra_message_with_place(Variable.ur3_to_device_up_plan_2)
        ur3_add_bowl_to_device_plan_3, ur3_place_loc_3 = self.read_tra_message_with_place(Variable.ur3_to_device_up_plan_3)
        ur3_add_bowl_to_device_plan_4, ur3_place_loc_4 = self.read_tra_message_with_place(Variable.ur3_to_device_up_plan_4)
        ur3_add_bowl_to_device_plan = [ur3_add_bowl_to_device_plan_2, ur3_add_bowl_to_device_plan_3, ur3_add_bowl_to_device_plan_4]
        ur3_place_bowl_to_device_loc = [ur3_place_loc_2, ur3_place_loc_3, ur3_place_loc_4]

        ur3_pull_dish_plan = self.read_tra_message(Variable.ur3_pull_dish_plan)

        ur3_get_start_plan = self.read_tra_message(Variable.ur3_get_start_plan)
        ur3_arm_ag95.execute(ur3_get_start_plan)

        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()

        order_num = 0
        no_order_time = 0

        self.have_plate = True

        # 接收订单
        for i in range(2):
            foodAndNum, order_id = acquire_orders_and_feedback.acquire_orders()
            foodAndNum = 'oo'
            order_id = 1
            if foodAndNum != 'no order' and order_id != 0:
                order_num += 1
                dish_num = len(foodAndNum)
                bowl_num = 0
                dish_list = []

                # for i in range(dish_num):
                #     bowl_num += foodAndNum[foodAndNum.keys()[i]]
                #     for _ in range(foodAndNum[foodAndNum.keys()[i]]):
                #         dish_list.append(str(foodAndNum.keys()[i]).replace('u\'', '\''))
                #
                # ordef_list = [bowl_num, dish_list]

                #一个订单中需要碗的数量
                # ordef_list = [[5,['Candy', 'Peanut', 'Candy', 'SunflowerSeed', 'Peanut']], [3, ['Candy', 'Peanut', 'SunflowerSeed']]]
                ordef_list = [1, ['SunflowerSeed']]

                need_num_bowl = ordef_list[0]
                dish_list = ordef_list[1]

                if need_num_bowl == len(dish_list):
                    print "-------------------------------------------------------------------------------"
                    print "执行第{}个订单".format(order_num)
                    # acquire_orders_and_feedback.set_order_running(order_id)
                    bowl_loc, bowl_up_loc = self.generate_bowl_loc(need_num_bowl)

                    all_dish_location_dictionary = self.read_all_dish_location_dictionary()

                    if self.have_plate == True:
                        signal_plate_receive.send('wait plate')
                        signal_plate_ready = signal_plate_receive.recv()
                        convey_tranfer_tray_dis = all_dish_location_dictionary[dish_list[0]] - 1.5
                        drop_bowl.control_convey_belt(convey_tranfer_tray_dis, convey_mqtt_client, False)
                    current_convey_location = all_dish_location_dictionary[dish_list[0]]

                    signal_3_car_send.send("have new orders")
                    signal_3_car_send.send(need_num_bowl)

                    for i in range(need_num_bowl):

                        signal_3_car_send.send(dish_list[i])

                        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
                        ur3_arm_ag95.go()

                        # ur3_arm_ag95.set_joint_value_target(UR3_get_bowl_loction)
                        ur3_arm_ag95.execute(ur3_get_bowl_plan)

                        rospy.sleep(0.2)

                        grippers_control.AG95_gripper(100, 0, AG95_client)
                        drop_finish = drop_bowl.send_order_drop_bowl(bowl_drop_mqtt_client)

                        if drop_finish == True:
                            ur3_arm_ag95.set_joint_value_target(Variable.ur3_lower_loc)
                            ur3_arm_ag95.go()

                            dish_dig_finish = signal_ur10_receive.recv()

                            if dish_dig_finish == "get to pull loc":
                                ur3_arm_ag95.set_joint_value_target(UR3_lay_location)
                                ur3_arm_ag95.go()

                                signal_ur3_send.send("bowl is ready")
                                start_dish_signal = signal_ur10_receive.recv()
                                if start_dish_signal == "start pull dish":
                                    ur3_arm_ag95.execute(ur3_pull_dish_plan)

                                    ur3_arm_ag95.set_joint_value_target(bowl_loc[i])
                                    ur3_arm_ag95.go()

                                    grippers_control.AG95_gripper(100, 55, AG95_client)

                                    rospy.sleep(0.2)
                                    ur3_arm_ag95.set_joint_value_target(bowl_up_loc[i])
                                    ur3_arm_ag95.go()

                                    if i == need_num_bowl - 1:
                                        # distance_exit = all_dish_location_dictionary[dish_list[-1]] - 0.18*need_num_bowl
                                        if self.have_plate == True:
                                            distance_exit = 1.5 - current_convey_location
                                        else:
                                            distance_exit = 1.65 - current_convey_location
                                        drop_bowl.control_convey_belt(distance_exit, bowl_drop_mqtt_client, False)
                                    elif i % 2 == 0 and i != need_num_bowl-1:
                                        next_distance = all_dish_location_dictionary[dish_list[i + 1]] - current_convey_location
                                        drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
                                        current_convey_location = current_convey_location + next_distance

                                    elif i % 2 == 1 and i != need_num_bowl-1:
                                        next_distance = -0.145 + all_dish_location_dictionary[dish_list[i+1]] - current_convey_location
                                        drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
                                        current_convey_location = current_convey_location + next_distance + 0.145

                    ur3_arm_ag95.set_joint_value_target(UR3_Q1)
                    ur3_arm_ag95.go()

                    # acquire_orders_and_feedback.set_order_finish(order_id)
                    no_order_time = 0
                    print "----------------------------------------------------------------------"
                else:
                    print "error oders, please check!"
            # 复位信号
            else:
                print 'no order, acquire again'
                no_order_time += 1
                rospy.sleep(1)
                if no_order_time == 10:
                    no_order_time = 0
                    signal_3_car_send.send("go home")
                    break

        signal_3_car_send.send("go home")
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)

    def move_car(self, signal_3_car_receive, signal_ur10_receive, signal_dig_start_send):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("Car_Move", anonymous=True, disable_signals=True)
        global dish_interval
        dish_interval = 0.35
        current_car_location = 0

        car_mqtt_client.connect(HOST, PORT, 60)
        car_mqtt_client.loop_start()
        all_dish_location_dictionary = self.read_all_dish_location_dictionary()
        while rospy.is_shutdown() == False:
            next_order_mess = signal_3_car_receive.recv()
            if next_order_mess == "have new orders":
                num_bowl = signal_3_car_receive.recv()
                for _ in range(num_bowl):
                    order_dish_name = signal_3_car_receive.recv()

                    dish_location = all_dish_location_dictionary[order_dish_name]

                    target_move_distance = dish_location - current_car_location

                    scan_dish.car_move(target_move_distance, car_mqtt_client)
                    print 'car move finish'
                    current_car_location += target_move_distance
                    print ("小车当前位置： " + str(current_car_location) + " m.")

                    signal_dig_start_send.send("dig start")

            elif next_order_mess == "go home":
                print "go home"
                scan_dish.car_move(-current_car_location, car_mqtt_client)
                current_car_location += -current_car_location
                print ("小车当前位置： " + str(current_car_location) + " m.")



                    # is_dig_complete = signal_ur10_receive.recv()

    def generate_bowl_loc(self, bowl_num):
        bowl_loc = []
        bowl_up_loc = []
        if bowl_num == 1:
            bowl_loc = one_bowl_loc
            bowl_up_loc = one_bowl_up_loc
        if bowl_num % 2 == 0:
            ite = bowl_num / 2
            for i in range(ite):
                bowl_loc.append(two_bowl_loc[0])
                bowl_loc.append(two_bowl_loc[1])
                bowl_up_loc.append(two_bowl_up_loc[0])
                bowl_up_loc.append(two_bowl_up_loc[1])
        elif bowl_num % 2 == 1:
            ite = bowl_num / 2
            for i in range(ite):
                bowl_loc.append(two_bowl_loc[0])
                bowl_loc.append(two_bowl_loc[1])
                bowl_up_loc.append(two_bowl_up_loc[0])
                bowl_up_loc.append(two_bowl_up_loc[1])
            bowl_loc.append(one_bowl_loc[0])
            bowl_up_loc.append((one_bowl_up_loc[0]))
        return bowl_loc, bowl_up_loc

    def read_all_dish_location_dictionary(self):
        # fw = open("/home/robot/photo/memu_weizhi_test.txt", "r+")
        # all_dish_location_dictionary = {}
        # for line in fw:
        #     result = eval(line)
        #     # print result
        #     only_food_name = result.keys()[0]
        #     all_dish_location_dictionary[only_food_name] = result[only_food_name][2][0]
        # # print all_dish_location_dictionary
        # return all_dish_location_dictionary
        fw = open("/home/robot/photo/memu_weizhi_test.txt", "r")
        content = fw.read()
        result = eval(content)
        return result

    #更新小车位置
    def car_move_update_location(self, distance, mqtt_client):
        scan_dish.car_move(distance,mqtt_client)
        self.current_car_location += distance
        print ("小车当前位置： " + str(self.current_car_location) + " m.")

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
        final_ur3ur10_alone_work()


    except rospy.ROSInterruptException:
        pass