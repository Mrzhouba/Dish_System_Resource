#!/usr/bin/env python
#coding=UTF-8
import tf
import rospy
import moveit_commander
import roslib;
import paho.mqtt.client as mqtt
import time
import json
import ctypes, inspect
import actionlib
import sys,signal, os
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/assist_modules')
import multiprocessing, threading

sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/pointnet2_pytorch/utils')

import psutil
import serial, binascii
import argparse
import Variable
from playsound import playsound
from sensor_msgs.msg import JointState,Image,CameraInfo, PointCloud2

from trajectory_msgs.msg import *
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from PyQt5.QtCore import *


from grippers_control import grippers_control
from scan_dish import scan_dish
from serial_drop_bowl import drop_bowl
from acquire_orders import acquire_orders_and_feedback
from final_judge_dish_location_qt import final_judge_dish_location_qt
from add_bowl_action import ur3_add_bowl_action


grippers_control = grippers_control()
scan_dish = scan_dish()
drop_bowl = drop_bowl()
acquire_orders_and_feedback = acquire_orders_and_feedback()
final_judge_dish_location_qt = final_judge_dish_location_qt()
ur3_add_bowl_action = ur3_add_bowl_action()

HOST = Variable.HOST
PORT = Variable.PORT
car_mqtt_client = mqtt.Client("control_car")
bowl_drop_mqtt_client = mqtt.Client("control_bowl_drop")
convey_mqtt_client = mqtt.Client("control_convey")

#拍摄准备点
UR10_Q1_to_left = Variable.UR10_Q1_to_left

#准备点
UR3_Q1 = Variable.UR3_Q1

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

point_network_args = parser.parse_args()

order_usb_name = Variable.order_usb_name


class final_ur3ur10_alone_work_qt(QThread):
    get_order_signal = pyqtSignal(str)
    # start_deal_signal = pyqtSignal()
    # end_deal_signal = pyqtSignal()
    work_status_signal = pyqtSignal(int)
    def __init__(self, parent=None):
        super(final_ur3ur10_alone_work_qt, self).__init__(parent)
        self.status = -1
        dish_location_relative_odom = {}
        fn = sys.stdin.fileno()
        self.current_car_location = 0
        self.judge_push = False
        self.dish_box_depth_in_camera = 0.6

        self.signal_stop_send, self.signal_stop_receive = multiprocessing.Pipe(True)
        self.signal_order_mess_send, self.signal_order_mess_receive = multiprocessing.Pipe(True)


        # signal_ur3_send, signal_ur3_receive = multiprocessing.Pipe(True)
        # signal_ur10_send, signal_ur10_receive = multiprocessing.Pipe(True)
        # signal_dig_start_send, signal_dig_start_receive = multiprocessing.Pipe(True)
        #
        # signal_3_car_send, signal_3_car_receive = multiprocessing.Pipe(True)
        # # signal_10_car_send, signal_10_car_receive = multiprocessing.Pipe(True)
        #
        # self.move_3 = multiprocessing.Process(target=self.move_ur3, args=(signal_ur3_send, signal_ur10_receive, signal_3_car_send,))
        # # self.move_10 = multiprocessing.Process(target=self.move_ur10, args=(signal_ur3_receive, signal_ur10_send, signal_dig_start_receive, ))
        # self.move_car = multiprocessing.Process(target=self.move_car, args=(signal_3_car_receive, signal_ur10_receive, signal_dig_start_send, ))

    def run(self):
        print 'run'

        signal_ur3_send, signal_ur3_receive = multiprocessing.Pipe(True)
        signal_ur10_send, signal_ur10_receive = multiprocessing.Pipe(True)
        signal_dig_start_send, signal_dig_start_receive = multiprocessing.Pipe(True)

        signal_3_car_send, signal_3_car_receive = multiprocessing.Pipe(True)
        # signal_10_car_send, signal_10_car_receive = multiprocessing.Pipe(True)
        signal_plate_send, signal_plate_receive = multiprocessing.Pipe(True)

        move_3 = multiprocessing.Process(target=self.move_ur3, args=(signal_ur3_send, signal_ur10_receive, signal_3_car_send,
                                                                    self.signal_order_mess_send, self.signal_stop_receive, signal_plate_receive))
        move_10 = multiprocessing.Process(target=self.move_ur10, args=(signal_ur3_receive, signal_ur10_send, signal_dig_start_receive, self.signal_stop_receive, ))
        move_car = multiprocessing.Process(target=self.move_car, args=(signal_3_car_receive, signal_ur10_receive, signal_dig_start_send,))
        monitor_plate = multiprocessing.Process(target=self.monitor_plate, args=(signal_plate_send,))

        move_3.start()
        move_3_pid = move_3.pid
        move_10.start()
        move_10_pid = move_10.pid
        move_car.start()
        move_car_pid = move_car.pid
        monitor_plate.start()
        monitor_plate_pid = monitor_plate.pid

        pause_process = multiprocessing.Process(target=self.pause_process, args=(move_10_pid, move_3_pid, move_car_pid, monitor_plate_pid,
                                                                                 self.signal_stop_send))
        pause_process.start()

        work_status = threading.Thread(target=self.status_feedback)
        order_mess = threading.Thread(target=self.order_feedback)

        work_status.start()
        order_mess.start()

        self.work_status_signal.emit(0)

        work_status.join()
        # play_sound.join()
        # print "end"
        # order_mess.join()
        # print "end thread"
        move_3.join()
        move_10.join()
        move_car.join()
        pause_process.join()

        print "end dig dish process"


    def status_feedback(self):
        while True:
            self.status = self.signal_stop_receive.recv()
            # print 123
            self.work_status_signal.emit(self.status)
            if self.status == 3:
                break

    def order_feedback(self):
        while True:
            # print self.status
            if self.status != 3:
                order_mess = self.signal_order_mess_receive.recv()
                # print "order"
                # print type(order_mess)
                if type(order_mess) != str:
                    self.get_order_signal.emit("receive new order" + str(order_mess))
                else:
                    self.get_order_signal.emit(str(order_mess))
            else:
                self.status = -1
                break


    def receive_stop_signal(self, signal):
        self.signal_stop_receive.send(signal)

    def receive_choose_dish_oeder(self, order_list):
        self.signal_order_mess_receive.send(order_list)


    def pause_process(self, move_10_pid, move_3_pid, move_car_pid, monitor_plate_pid, signal_stop_send):

        ur3_pause = psutil.Process(move_3_pid)
        ur10_pause = psutil.Process(move_10_pid)
        car_pause = psutil.Process(move_car_pid)
        plate_pause = psutil.Process(monitor_plate_pid)

        while True:
            is_stop_signal = signal_stop_send.recv()
            if is_stop_signal == 'stop':
                ur3_pause.suspend()
                ur10_pause.suspend()
                car_pause.suspend()

                print('打菜暂停运行')
                signal_stop_send.send(1)

            elif is_stop_signal == 'resume':
                ur3_pause.resume()
                ur10_pause.resume()
                car_pause.resume()

                print('打菜恢复运行')
                signal_stop_send.send(2)

            elif is_stop_signal == 'kill':
                ur3_pause.kill()
                ur10_pause.kill()
                car_pause.kill()
                plate_pause.kill()

                print('打菜结束运行')
                signal_stop_send.send(3)
                break

            elif is_stop_signal == 'add_stop':
                ur3_pause.suspend()
                ur10_pause.suspend()
                car_pause.suspend()

                print('需要补充菜品')
                signal_stop_send.send(4)

            elif is_stop_signal == 'button note':
                print('按键提示')
                signal_stop_send.send(5)

    def monitor_plate(self, signal_plate_send):
        portx = order_usb_name
        bps = 9600
        timex = None

        while 1:
            ur3_request = signal_plate_send.recv()
            if ur3_request == 'wait plate':
                ser = serial.Serial(portx, bps, timeout=timex)
                current_time = time.time()
                while 1:
                    data = binascii.b2a_hex(ser.read())
                    if time.time() - current_time > 0.5:
                        # data = binascii.b2a_hex(ser.read())
                        if data == '03':
                            signal_plate_send.send("plate is ready")
                            ser.close()
                            break



    def move_ur10(self, signal_ur3_receive, signal_ur10_send, signal_dig_start_receive, signal_stop_receive):
        from get_model_output import get_model_output
        self.get_model_output = get_model_output(point_network_args.model_name, point_network_args.channels,
                                                 point_network_args.nclasses,
                                                 point_network_args.weight_path)
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("dig_dish", anonymous=True, disable_signals=True)
        reference_frame = "body_link"
        ur10_group_name = "ur10_with_gripper"
        ur10_arm_spoon = moveit_commander.MoveGroupCommander(ur10_group_name, ns='ur10')

        # 当运动规划失败后，允许重新规划
        ur10_arm_spoon.allow_replanning(False)
        ur10_arm_spoon.set_planning_time(1.0)

        # 设置目标位置所使用的参考坐标系
        ur10_arm_spoon.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        ur10_arm_spoon.set_goal_position_tolerance(0.001)
        ur10_arm_spoon.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        ur10_arm_spoon.set_max_acceleration_scaling_factor(0.5)
        ur10_arm_spoon.set_max_velocity_scaling_factor(0.5)

        # 获取终端link的名称
        ur10_spoon_end_effector_link = ur10_arm_spoon.get_end_effector_link()

        ur10_arm_spoon.set_joint_value_target(UR10_Q1_to_left)
        ur10_arm_spoon.go()

        #获取菜品位置
        dish_location_relative_odom = self.read_all_dish_location_dictionary()
        args = [ur10_arm_spoon, ur10_spoon_end_effector_link, signal_ur10_send, signal_ur3_receive,
                self.get_model_output, signal_stop_receive]
        while not rospy.is_shutdown():
            start_signal = signal_dig_start_receive.recv()
            signal_list = start_signal.split(',')
            completed = False
            is_need_push = False
            if signal_list[0] == "dig start":
                print "当前打取菜品", signal_list[1]
                while not completed:
                    # print 1234
                    point_cloud_data = rospy.wait_for_message("/camera2/depth/color/points", PointCloud2, timeout=1)
                    completed, is_need_push = final_judge_dish_location_qt.points_callback(point_cloud_data, args, is_need_push, signal_list[1])
                    print completed
            # self.point_sub = rospy.Subscriber("/camera2/depth/color/points", PointCloud2, final_judge_dish_location_qt.points_callback,
            #                                   (ur10_arm_spoon, ur10_spoon_end_effector_link, signal_ur10_send, signal_ur3_receive, signal_dig_start_receive))
            # rospy.spin()

    def move_ur3(self, signal_ur3_send, signal_ur10_receive, signal_3_car_send, signal_order_mess_send, signal_stop_receive, signal_plate_receive):
        # while 1:
        #     rospy.sleep(1)
        #     foodAndNum, order_id = acquire_orders_and_feedback.acquire_orders()
        #     signal_order_mess_send.send(foodAndNum)
        #     # print foodAndNum
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

        ur3_get_start_plan = self.read_tra_message(
            '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/'
            'saves_trajectory_mess/get_bowl_trajectory/ur3_to_start_trajectory.json')
        # ur3_arm_ag95.execute(ur3_get_start_plan)

        self.add_bowl_index = 0

        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()

        dish_work_type = signal_order_mess_send.recv()
        if len(dish_work_type) == 1:
            cycle_num = 3600
            work_type = "auto order"
            print "打菜工作开始前碗数量", dish_work_type[0]
            current_bowl_num = dish_work_type[0]
        else:
            cycle_num = 1
            work_type = "manual order"

        order_num = 0
        no_order_time = 0

        all_dish_location_dictionary = self.read_all_dish_location_dictionary()
        # 接收订单
        for i in range(cycle_num):
            # print foodAndNum
            # print type(foodAndNum)
            is_vaild = True
            if work_type == "auto order":
                foodAndNum, order_id = acquire_orders_and_feedback.acquire_orders()
                if foodAndNum != 'no order' and order_id != 0:
                    signal_order_mess_send.send(foodAndNum)
                    order_num += 1
                    dish_num = len(foodAndNum)
                    bowl_num = 0
                    dish_list = []

                    for i in range(dish_num):
                        bowl_num += foodAndNum[foodAndNum.keys()[i]]
                        for _ in range(foodAndNum[foodAndNum.keys()[i]]):
                            dish_list.append(str(foodAndNum.keys()[i]).replace('u\'', '\''))

                    ordef_list = [bowl_num, dish_list]
                    for name in dish_list:
                        if all_dish_location_dictionary.get(name) == None:
                            is_vaild = False
                            break

                else:
                    ordef_list = []
            else:
                ordef_list = dish_work_type

            if is_vaild == False:
                ordef_list = []
                acquire_orders_and_feedback.set_order_running(order_id)
                signal_order_mess_send.send("收到无效订单")

            if len(ordef_list) != 0:
                need_num_bowl = ordef_list[0]
                dish_list = ordef_list[1]

                if need_num_bowl == len(dish_list):
                    # current_car_loc = signal_3_car_send.recv()
                    # dish_list = self.change_dish_list(current_car_loc, dish_list)
                    # print "sort_dish", dish_list

                    print "-------------------------------------------------------------------------------"
                    print "执行第{}个订单".format(order_num)
                    if work_type == "auto order":
                        acquire_orders_and_feedback.set_order_running(order_id)
                        signal_order_mess_send.send(str(order_id) + ":开始处理")
                    else:
                        signal_order_mess_send.send("开始处理")

                    bowl_loc, bowl_up_loc = self.generate_bowl_loc(need_num_bowl)

                    # signal_plate_receive.send('wait plate')
                    # signal_stop_receive.send('button note')
                    # signal_plate_ready = signal_plate_receive.recv()

                    # convey_tranfer_tray_dis = all_dish_location_dictionary[dish_list[0]] - 1.4
                    # drop_bowl.control_convey_belt(convey_tranfer_tray_dis, convey_mqtt_client, False)
                    # current_convey_location = all_dish_location_dictionary[dish_list[0]]

                    signal_3_car_send.send("have new orders")
                    signal_3_car_send.send(need_num_bowl)
                    current_car_loc = signal_3_car_send.recv()
                    dish_list = self.change_dish_list(current_car_loc, dish_list)
                    print "sort_dish", dish_list

                    signal_plate_receive.send('wait plate')
                    signal_stop_receive.send('button note')
                    signal_plate_ready = signal_plate_receive.recv()

                    convey_tranfer_tray_dis = all_dish_location_dictionary[dish_list[0]] - 1.44
                    drop_bowl.control_convey_belt(convey_tranfer_tray_dis, convey_mqtt_client, False)
                    current_convey_location = all_dish_location_dictionary[dish_list[0]]

                    convey_distance_exit = 0
                    for i in range(need_num_bowl):
                        if work_type == 'auto order':
                            if current_bowl_num < 3 and self.add_bowl_index <= 1:
                                ur3_arm_ag95.set_joint_value_target(UR3_Q1)
                                ur3_arm_ag95.go()
                                signal_order_mess_send.send("开始补碗")

                                signal_3_car_send.send("add bowl")
                                add_bowl_finish_signal = self.ur3_process_add_bowl(ur3_arm_ag95, signal_3_car_send, AG95_client)

                                if add_bowl_finish_signal == True:
                                    signal_order_mess_send.send("补碗完成")
                                    current_bowl_num = current_bowl_num + 7


                        if work_type == 'auto order':
                            current_bowl_num = current_bowl_num - 1

                        signal_3_car_send.send(dish_list[i])

                        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
                        ur3_arm_ag95.go()

                        # 读轨迹
                        ur3_arm_ag95.execute(ur3_get_bowl_plan)

                        # 两次set取碗
                        # ur3_arm_ag95.set_joint_value_target(Variable.ur3_middle_to_get_bowl_set)
                        # ur3_arm_ag95.go()
                        #
                        # ur3_arm_ag95.set_joint_value_target(Variable.ur3_end_to_get_bowl_set)
                        # ur3_arm_ag95.go()

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

                            # 同时到达
                            signal_ur3_send.send("drop bowl finish")

                            dish_dig_finish = signal_ur10_receive.recv()

                            if dish_dig_finish == "get to pull loc":
                                ur3_arm_ag95.set_joint_value_target(UR3_lay_location)
                                ur3_arm_ag95.go()

                                signal_ur3_send.send("bowl is ready")
                                start_dish_signal = signal_ur10_receive.recv()
                                if start_dish_signal == "start pull dish":
                                    ur3_arm_ag95.execute(ur3_pull_dish_plan)

                                    shake_is_finish = signal_ur10_receive.recv()
                                    if shake_is_finish == "shake finish":

                                        ur3_arm_ag95.set_joint_value_target(bowl_loc[i])
                                        ur3_arm_ag95.go()

                                        grippers_control.AG95_gripper(100, 55, AG95_client)

                                        rospy.sleep(0.2)
                                        ur3_arm_ag95.set_joint_value_target(bowl_up_loc[i])
                                        ur3_arm_ag95.go()

                                        if i == need_num_bowl - 1:
                                            # distance_exit = all_dish_location_dictionary[dish_list[-1]] - 0.18*need_num_bowl
                                            distance_exit = 1.44 - current_convey_location
                                            convey_distance_exit = distance_exit
                                            drop_bowl.control_convey_belt(distance_exit, convey_mqtt_client, False)
                                        # elif i % 2 == 0 and i != need_num_bowl-1 and i != 2:
                                        #     next_distance = all_dish_location_dictionary[dish_list[i + 1]] - current_convey_location
                                        #     drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
                                        #     current_convey_location = current_convey_location + next_distance
                                        # elif i == 2:
                                        #     next_distance = -0.145 + all_dish_location_dictionary[dish_list[i + 1]] - current_convey_location
                                        #     drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
                                        #     current_convey_location = current_convey_location + next_distance
                                        elif i % 2 == 0 and i != need_num_bowl-1:
                                            if i >= 2:
                                                next_distance = -0.145 + all_dish_location_dictionary[dish_list[i + 1]] - current_convey_location
                                            else:
                                                next_distance = all_dish_location_dictionary[dish_list[i + 1]] - current_convey_location
                                            drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
                                            current_convey_location = current_convey_location + next_distance
                                        elif i % 2 == 1 and i != need_num_bowl-1:
                                            next_distance = -0.145 + all_dish_location_dictionary[dish_list[i+1]] - current_convey_location

                                            drop_bowl.control_convey_belt(next_distance, convey_mqtt_client, False)
                                            current_convey_location = current_convey_location + next_distance


                    ur3_arm_ag95.set_joint_value_target(UR3_Q1)
                    ur3_arm_ag95.go()
                    print 'convey_distance_exit', convey_distance_exit
                    if convey_distance_exit == 1.44+0.145:
                        rospy.sleep(5)
                    elif convey_distance_exit == 1.08+0.145:
                        rospy.sleep(4)
                    elif convey_distance_exit == 0.72+0.145:
                        rospy.sleep(3)
                    elif convey_distance_exit == 0.36+0.145:
                        rospy.sleep(2)
                    else:
                        rospy.sleep(1)

                    if work_type == "auto order":
                        acquire_orders_and_feedback.set_order_finish(order_id)
                        signal_order_mess_send.send(str(order_id) + ":处理结束")
                    else:
                        signal_order_mess_send.send("处理结束")
                    no_order_time = 0
                    print "----------------------------------------------------------------------"
                else:
                    print "error oders, please check!"
            # 复位信号
            else:
                no_order_time += 1
                rospy.sleep(1)
                if no_order_time % 5 == 0:
                    print 'no order, acquire again'
                    signal_order_mess_send.send("暂无订单")
                if no_order_time == 180:
                    no_order_time = 0
                    signal_3_car_send.send("go home")
                    break

        if work_type == "manual order":
            signal_3_car_send.send("go home")
        else:
            signal_order_mess_send.send("长时间无订单,结束打菜")

        is_move_over = signal_3_car_send.recv()
        print is_move_over
        signal_stop_receive.send('kill')

        # signal_3_car_send.send("go home")
        # moveit_commander.roscpp_shutdown()
        # moveit_commander.os._exit(0)


    def move_car(self, signal_3_car_receive, signal_ur10_receive, signal_dig_start_send):
        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("Car_Move", anonymous=True, disable_signals=True)
        global dish_interval
        dish_interval = 0.35
        current_car_location = 0

        car_mqtt_client.connect(HOST, PORT, 60)
        car_mqtt_client.loop_start()
        all_dish_location_dictionary = self.read_all_dish_location_dictionary()
        while rospy.is_shutdown() == False:
            # signal_3_car_receive.send(current_car_location)
            next_order_mess = signal_3_car_receive.recv()
            print next_order_mess
            if next_order_mess == "have new orders":
                num_bowl = signal_3_car_receive.recv()
                print num_bowl
                signal_3_car_receive.send(current_car_location)

                for _ in range(num_bowl):
                    order_dish_name = signal_3_car_receive.recv()
                    print order_dish_name
                    if order_dish_name == 'add bowl':
                        # ur3_add_bowl_action.ur3_add_bowl_work(current_car_location)
                        move_dis = 0.36 - current_car_location
                        scan_dish.car_move(move_dis, car_mqtt_client)
                        current_car_location += move_dis
                        signal_3_car_receive.send("arrive target")

                        # 小车回到补碗前的位置
                        # go_back_signal = signal_3_car_receive.recv()
                        # if go_back_signal == 'go back':
                        #     scan_dish.car_move(-move_dis, car_mqtt_client)
                        order_dish_name = signal_3_car_receive.recv()

                    dish_location = all_dish_location_dictionary[order_dish_name]

                    target_move_distance = dish_location - current_car_location

                    scan_dish.car_move(target_move_distance, car_mqtt_client)
                    current_car_location += target_move_distance
                    print ("小车当前位置： " + str(current_car_location) + " m.")
                    signal = "dig start," + order_dish_name
                    print signal
                    signal_dig_start_send.send(signal)

            elif next_order_mess == "go home":
                print "go home 11"
                scan_dish.car_move(-current_car_location, car_mqtt_client)
                current_car_location += -current_car_location
                print ("小车当前位置： " + str(current_car_location) + " m.")
                signal_3_car_receive.send("move over")

            # elif next_order_mess == "add bowl":
            #     ur3_add_bowl_action.ur3_add_bowl_work(current_car_location)
            #     signal_3_car_receive.send("add bowl finish")
                    # is_dig_complete = signal_ur10_receive.recv()

    def ur3_process_add_bowl(self, Group, signal_3_car_send, AG95_client):
        # ur3_add_bowl_plan = self.read_tra_message(
        #     '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
        #     '/saves_trajectory_mess/add_bowl_trajectory/xinchang_ur3_to_get_bowl_plan.json')
        # ur3_add_bowl_to_device_plan_2, ur3_place_loc_2 = self.read_tra_message_with_place(
        #     '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
        #     '/saves_trajectory_mess/add_bowl_trajectory/xinchang_ur3_to_device_up_plan_2_with_loc.json')
        # ur3_add_bowl_to_device_plan_3, ur3_place_loc_3 = self.read_tra_message_with_place(
        #     '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
        #     '/saves_trajectory_mess/add_bowl_trajectory/ur3_to_device_up_plan_3.json')
        # ur3_add_bowl_to_device_plan_4, ur3_place_loc_4 = self.read_tra_message_with_place(
        #     '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts'
        #     '/saves_trajectory_mess/add_bowl_trajectory/ur3_to_device_up_plan_4.json')
        # ur3_add_bowl_to_device_plan = [ur3_add_bowl_to_device_plan_2, ur3_add_bowl_to_device_plan_3,
        #                                ur3_add_bowl_to_device_plan_4]
        # ur3_place_bowl_to_device_loc = [ur3_place_loc_2, ur3_place_loc_3, ur3_place_loc_4]

        # add_bowl_signal = signal_3_car_send.recv()
        # if add_bowl_signal == 'arrive target':
        #     grippers_control.AG95_gripper(100, 100, AG95_client)
        #
        #     Group.execute(ur3_add_bowl_plan)
        #     rospy.sleep(0.2)
        #     grippers_control.AG95_gripper(100, 15, AG95_client)
        #
        #     Group.execute(ur3_add_bowl_to_device_plan_2)
        #
        #     Group.set_joint_value_target(ur3_place_loc_2)
        #     Group.go()
        #
        #     rospy.sleep(0.5)
        #     grippers_control.AG95_gripper(100, 88, AG95_client)
        #     #
        #     signal_3_car_send.send('go back')
        #     Group.set_joint_value_target(UR3_Q1)
        #     Group.go()
        #
        #     return True
        ur3_to_get_bowl_ready_loc = Variable.ur3_multi_catch_bowl_ready_location[self.add_bowl_index % 2]
        ur3_to_get_bowl_loc = Variable.ur3_multi_catch_bowl_location[self.add_bowl_index % 2]
        ur3_add_bowl_to_device_plan_2, ur3_place_loc_2 = self.read_tra_message_with_place(Variable.ur3_multi_to_device_up_plan_2[self.add_bowl_index % 2])
        add_bowl_signal = signal_3_car_send.recv()
        if add_bowl_signal == 'arrive target':
            grippers_control.AG95_gripper(100, 100, AG95_client)

            Group.set_joint_value_target(ur3_to_get_bowl_ready_loc)
            Group.go()

            Group.set_joint_value_target(ur3_to_get_bowl_loc)
            Group.go()

            rospy.sleep(0.2)
            grippers_control.AG95_gripper(100, 15, AG95_client)

            Group.execute(ur3_add_bowl_to_device_plan_2)

            Group.set_joint_value_target(ur3_place_loc_2)
            Group.go()

            rospy.sleep(0.5)
            grippers_control.AG95_gripper(100, 88, AG95_client)

            # 小车回到补碗前的位置
            # signal_3_car_send.send('go back')

            Group.set_joint_value_target(UR3_Q1)
            Group.go()
            self.add_bowl_index += 1
            return True


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

    def change_dish_list(self, current_car_loc, dish_list):
        all_dish_location_dictionary = self.read_all_dish_location_dictionary()
        need_location = []
        for dish_name in dish_list:
            need_location.append(all_dish_location_dictionary[dish_name])
        sort_dish_list = []
        dish_num = len(dish_list)
        dish_list_copy = dish_list
        for _ in range(dish_num):
            maxDIs = 5
            next_dish_name = ""
            for name in dish_list_copy:
                curDis = abs(current_car_loc - all_dish_location_dictionary[name])
                if maxDIs > curDis:
                    maxDIs = curDis
                    next_dish_name = name

            sort_dish_list.append(next_dish_name)
            dish_list_copy.remove(next_dish_name)
            current_car_loc = all_dish_location_dictionary[next_dish_name]

        return sort_dish_list









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

    def read_tra_message_with_place(self, file_path):
        file = open(file_path)
        trajectory_mess = json.load(file)
        # print trajectory_mess
        num_points = len(trajectory_mess)
        plan = RobotTrajectory()
        plan.joint_trajectory.joint_names = trajectory_mess[0]['joint_names']
        ur3_bowl_place_loc = trajectory_mess[-1]['ur3_place_loc']
        plan.joint_trajectory.header.frame_id = '/body_link'

        for i in range(1, num_points-1):
            point = JointTrajectoryPoint()
            point.positions = trajectory_mess[i]['position']
            point.velocities = trajectory_mess[i]['velocities']
            point.accelerations = trajectory_mess[i]['accelerations']
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
        # print len(result)
        filename = file_path
        with open(filename, 'w') as f:
            json.dump(result, f, ensure_ascii=False)


