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
import math
import numpy
import constant

from copy import deepcopy
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import PoseStamped, Pose, PointStamped
from sensor_msgs.msg import JointState,Image,CameraInfo, PointCloud2
from open3d_ros_helper import open3d_ros_helper as orh
from trajectory_msgs.msg import *
from dh_hand_driver.msg import ActuateHandAction, ActuateHandGoal

from judge_dish_location import judge_dish_location
from trajectory_plan import trajectory_plan
from grippers_control import grippers_control
from scan_dish import scan_dish
from serial_drop_bowl import drop_bowl
from acquire_orders import acquire_orders_and_feedback
#UR10_Q1 = [ -0.1647556463824671, -1.4304574171649378, 0.9727334976196289, -1.656503979359762, -1.4693434874164026, 2.215650796890259]
UR10_Q1 = [-0.2823393980609339, -1.4180730024920862, 0.9614772796630859, -1.717309300099508, -1.4651225248919886, 2.0386080741882324]
ur10_pull_ready = [-0.6376660505877894, -1.0643394629107874, 1.3383679389953613, -2.1668217817889612, -2.598771874104635, 2.0382609367370605]
ur10_pull = [-0.6584623495685022, -1.0669177214251917, 1.2827062606811523, -2.373713795338766, -2.504651133214132, 3.898932456970215]
ur10_test = [-0.2961519400226038, -1.4093807379352015, 1.9891185760498047, -3.4746907393084925, -1.4024160544024866, 0.7453573942184448]

UR3_Q1 = [-1.1703670660602015, -1.6691835562335413, -1.4985736052142542, 0.022022485733032227, 1.1698206663131714, 4.711615562438965]
# UR3_Q2为吸碗点
UR3_Q2 = [-0.5644510428058069, -1.6618087927447718, -1.583010498677389, 0.11027586460113525, 1.4915417432785034, 4.711711406707764]
# 过度点
UR3_Q3 = [0.5536326169967651, -1.660036865864889, -1.9021084944354456, 0.2982989549636841, 0.9849501252174377, 4.8075432777404785]
# 放碗点
UR3_Q4 = []
# 离开碗点

judge_dish_location = judge_dish_location()
trajectory_plan = trajectory_plan()
grippers_control = grippers_control()
scan_dish = scan_dish()
drop_bowl = drop_bowl()
acquire_orders_and_feedback = acquire_orders_and_feedback()

ur3_dish_pose = PoseStamped()

HOST = "192.168.1.176"
PORT = 1883
mqtt_client = mqtt.Client("ubuntu-PC-gripper-control")


dish_interval = None
class Get_location:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("broadcast_location", anonymous=True)

        ######################################################################
        # 初始化UR10规划组
        ur10_group_name = "ur10_with_gripper"
        ur10_arm_spoon = moveit_commander.MoveGroupCommander(ur10_group_name)

        # 当运动规划失败后，允许重新规划
        ur10_arm_spoon.allow_replanning(False)

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'body_link'
        ur10_arm_spoon.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        ur10_arm_spoon.set_goal_position_tolerance(0.001)
        ur10_arm_spoon.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        ur10_arm_spoon.set_max_acceleration_scaling_factor(0.1)
        ur10_arm_spoon.set_max_velocity_scaling_factor(0.1)

        ur10_spoon_end_effector_link = ur10_arm_spoon.get_end_effector_link()
        ur10_arm_spoon.set_planner_id("RRTConnect")

        ######################################################################
        # 初始化UR10规划组
        ur3_group_name = "ur3_with_gripper"
        ur3_arm_ag95 = moveit_commander.MoveGroupCommander(ur3_group_name)

        # 当运动规划失败后，允许重新规划
        ur3_arm_ag95.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        ur3_arm_ag95.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        ur3_arm_ag95.set_goal_position_tolerance(0.001)
        ur3_arm_ag95.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        ur3_arm_ag95.set_max_acceleration_scaling_factor(0.6)
        ur3_arm_ag95.set_max_velocity_scaling_factor(0.6)

        ur3_ag95_end_effector_link = ur3_arm_ag95.get_end_effector_link()
        ur3_arm_ag95.set_planner_id("RRTConnect")

        ######################################################################

        self.camera_info_sub = rospy.Subscriber("/camera2/aligned_depth_to_color/camera_info", CameraInfo, self.info_callback)
        self.camera_info = CameraInfo()
        self.bridge = CvBridge()
        self.image_depth_sub = rospy.Subscriber("/camera2/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        self.depth_image = Image()

        self.original_pointcloud = o3d.geometry.PointCloud()
        # self.dish_area_pointcloud = o3d.geometry.PointCloud()

        self.down_pointcloud = o3d.geometry.PointCloud()
        self.recognize_pointcloud = o3d.geometry.PointCloud()

        self.min_dian_index = 0
        self.max_dian_index = 0
        self.dish_box_depth_in_camera = 0.721
        self.judge_push = False

        self.start_rot_message = tf.TransformBroadcaster()
        self.end_rot_message = tf.TransformBroadcaster()
        self.intermediate_rot_message = tf.TransformBroadcaster()
        self.dish_up_pose = tf.TransformBroadcaster()

        dish_location_relative_odom = {}

        self.current_car_location = 0
        global dish_interval
        dish_interval = 0.35



        mqtt_client.connect(HOST, PORT, 60)
        mqtt_client.loop_start()

        # pull_quaternion = self.compute_rotate_matrix(25 * math.pi / 180, -2 * math.pi / 3, -math.pi / 2)
        # start_rot_message = tf.TransformBroadcaster()
        # while(1):
        #     start_rot_message.sendTransform((-1, -1, 1),
        #                                     pull_quaternion,
        #                                     rospy.Time.now(),
        #                                     "/test_loca", "/body_link")
        #
        # rospy.sleep(100000.0)
        # ur3_arm_ag95.go()

            # ur3_arm_ag95.set_joint_value_target(UR3_Q2)
            # ur3_arm_ag95.go()
            # ur3_arm_ag95.set_joint_value_target(UR3_Q3)
            # ur3_arm_ag95.go()

        # print ur3_arm_ag95.get_current_pose(ur3_ag95_end_effector_link)
        ur3_arm_ag95.clear_pose_target(ur3_ag95_end_effector_link)
        ur10_arm_spoon.clear_pose_target(ur10_spoon_end_effector_link)

        ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        ur3_arm_ag95.go()
        # drop_bowl.send_order_drop_bowl()
        # ur3_arm_ag95.set_pose_target(ur3_bowl_pose, ur3_ag95_end_effector_link)
        # ur3_arm_ag95.go()

        # ur3_arm_ag95.set_joint_value_target(UR3_Q1)
        # ur3_arm_ag95.go()
        # print ur3_arm_ag95.get_current_pose(ur3_ag95_end_effector_link)
        #
        #
        ur10_arm_spoon.set_joint_value_target(UR10_Q1)
        ur10_arm_spoon.go()
        # print ur10_arm_spoon.get_current_pose(ur10_spoon_end_effector_link)
        AG95_client = actionlib.SimpleActionClient('actuate_hand',ActuateHandAction)
        # AG95_client.wait_for_server()

        # self.bowl_catch(ur3_arm_ag95, ur3_ag95_end_effector_link, reference_frame, AG95_client, 2)
        # self.catch_dish(ur10_arm_spoon, ur10_spoon_end_effector_link, reference_frame)
        ######################################################################
        while not rospy.is_shutdown():
            # input_num = input("Function choose (1. scan_dish 2. move_together):\n")
            input_num = 0
            # print input_num
            if input_num == 1:
                print ("1. 创建菜单模式")
                scan_dish.car_set_zero(mqtt_client,HOST,PORT)

                dish_location_relative_odom = self.move_recognize(mqtt_client, 3, dish_interval)
                # scan_dish.send_memu()
                # scan_dish.car_move(0.05, mqtt_client)
                # scan_dish.car_move(-0.75, mqtt_client)
                # scan_dish.car_move(-1, mqtt_client)

                # scan_dish.car_move(0, mqtt_client)

                break
            elif input_num == 2:
                print ("2. 打菜模式")
                dish_location_relative_odom = self.read_all_dish_location_dictionary()
                # print ur3_arm_ag95.get_current_pose(ur3_ag95_end_effector_link)
                # print ur10_arm_spoon.get_current_pose(ur10_spoon_end_effector_link)
                foodAndNum = acquire_orders_and_feedback.acquire_orders()
                # foodAndNum.value()
                dish_num = len(foodAndNum)
                bowl_num = 0
                dish_list = []

                for i in range(dish_num):
                    bowl_num += foodAndNum[foodAndNum.keys()[i]]
                    dish_list.append(str(foodAndNum.keys()[i]).replace('u\'','\''))
                    # print foodAndNum[foodAndNum.keys()[1]]

                print ("碗的数量: " + str(bowl_num))
                print ("获取订单的内容： " + str(dish_list))
                # bowl_num = 1
                # dish_list = ['SunflowerSeed']
                dish_num = 1

                # 跑车取碗
                # self.car_move_update_location(3 * dish_interval, mqtt_client)

                # bowl_pose_list_xyz = self.bowl_catch(ur3_arm_ag95,ur3_ag95_end_effector_link,reference_frame,AG95_client,bowl_num)
                bowl_pose_list_xyz = [[1,1,1]]
                delt_dish_bowl = 0
                for i in range(dish_num):
                    # 判断车与目标距离
                    target_dish_location = dish_location_relative_odom[dish_list[i]]
                    target_move_distance = target_dish_location - self.current_car_location
                    print ("目标菜的距离： " + str(target_move_distance) + " m.")
                    self.car_move_update_location(target_move_distance, mqtt_client)
                    delt_dish_bowl += target_move_distance
                    #打菜
                    self.catch_dish(ur10_arm_spoon, ur10_spoon_end_effector_link, reference_frame)
                    # self.pull_dish_to_bowl(i, bowl_pose_list_xyz, ur10_arm_spoon, ur10_spoon_end_effector_link, reference_frame, delt_dish_bowl)
                    rospy.sleep(1)


                if_back = input("Go home?\n")
                if(if_back == 1):
                    self.car_move_update_location(-self.current_car_location, mqtt_client)
                else:
                    pass


                break
            elif input_num == 3:

                scan_dish.car_set_zero(mqtt_client, HOST, PORT)
                scan_dish.car_move(0.35, mqtt_client)

                break
            else:
                # print ("Error input!")
                pass






            # if(is_push):
            #     self.push_move_robot(ur10_arm_spoon, ur10_spoon_end_effector_link, push_start_pose, push_end_pose)
            # else:
            #     self.dig_move_robot(ur10_arm_spoon, ur10_spoon_end_effector_link, dish_start_pose, dish_intermediate_pose,dish_end_pose,dish_up_pose)
            # rospy.sleep(1)


        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    # def communication_callback(self, msg):
    #     print str(msg)
    #     if (str(msg) == "data: \"i need location\""):
    #         self.transform_to_origin()
    def read_all_dish_location_dictionary(self):
        fw = open("/home/robot/photo/memu_weizhi_test.txt", "r+")
        all_dish_location_dictionary = {}
        for line in fw:
            result = eval(line)
            # print result
            only_food_name = result.keys()[0]
            all_dish_location_dictionary[only_food_name] = result[only_food_name][2][0]
        # print all_dish_location_dictionary
        return all_dish_location_dictionary

    def pull_dish_to_bowl(self, num, list, Group, Group_end_effector, reference_frame, delt_distance):
        dish_pull_ready = PoseStamped()
        dish_pull_ready.header.frame_id = reference_frame
        dish_pull_ready.header.stamp = rospy.Time(0)
        dish_pull_ready.pose.position.x = list[num][0] + delt_distance
        dish_pull_ready.pose.position.y = list[num][1] - 0.07
        dish_pull_ready.pose.position.z = list[num][2] + 0.05

        rotate_z = self.convert_rpy(list[num][0], list[num][1])
        pull_ready_quaternion = tf.transformations.quaternion_from_euler(0, 0, -math.pi/2)
        dish_pull_ready.pose.orientation.x = pull_ready_quaternion[0]
        dish_pull_ready.pose.orientation.y = pull_ready_quaternion[1]
        dish_pull_ready.pose.orientation.z = pull_ready_quaternion[2]
        dish_pull_ready.pose.orientation.w = pull_ready_quaternion[3]

        dish_pull = PoseStamped()
        dish_pull.header.frame_id = reference_frame
        dish_pull.header.stamp = rospy.Time(0)
        dish_pull.pose.position.x = list[num][0] + delt_distance + 0.01
        dish_pull.pose.position.y = list[num][1] - 0.02
        dish_pull.pose.position.z = list[num][2] + 0.05

        pull_quaternion = self.compute_rotate_matrix(25 * math.pi / 180, -2 * math.pi / 3, -math.pi / 2)

        dish_pull.pose.orientation.x = pull_quaternion[0]
        dish_pull.pose.orientation.y = pull_quaternion[1]
        dish_pull.pose.orientation.z = pull_quaternion[2]
        dish_pull.pose.orientation.w = pull_quaternion[3]

        Group.set_pose_target(dish_pull_ready, Group_end_effector)
        Group.go()

        Group.set_pose_target(dish_pull, Group_end_effector)
        Group.go()

        Group.set_joint_value_target(UR10_Q1)
        Group.go()

    def compute_rotate_matrix(self, angle_x, angle_y, angle_z):
        matrix_z = [[math.cos(angle_z), -math.sin(angle_z), 0, 0],
                    [math.sin(angle_z), math.cos(angle_z), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        matrix_x = [[1, 0, 0, 0], [0, math.cos(angle_x), -math.sin(angle_x), 0],
                    [0, math.sin(angle_x), math.cos(angle_x), 0], [0, 0, 0, 1]]
        matrix_y = [[math.cos(angle_y), 0, math.sin(angle_y), 0], [0, 1, 0, 0],
                    [-math.sin(angle_y), 0, math.cos(angle_y), 0], [0, 0, 0, 1]]

        matrix_temp = numpy.dot(matrix_z, matrix_x)
        matrix = numpy.dot(matrix_temp, matrix_y)
        quaternion = tf.transformations.quaternion_from_matrix(matrix)
        return quaternion

    def car_move_update_location(self, distance, mqtt_client):
        scan_dish.car_move(distance,mqtt_client)
        self.current_car_location += distance
        print ("小车当前位置： " + str(self.current_car_location) + " m.")

    def catch_dish(self, Group, Group_end_effector, reference_frame):
        self.point_sub = rospy.Subscriber("/camera2/depth/color/points", PointCloud2, self.point_callback)
        rospy.sleep(0.5)

        try:

            if (self.judge_push):
                push_start_pose, push_end_pose = self.receive_push_pose(reference_frame)
                rospy.sleep(0.2)
                self.point_sub.unregister()
                # self.push_move_robot(ur10_arm_spoon, ur10_spoon_end_effector_link, push_start_pose, push_end_pose)

            else:

                dish_start_pose, dish_end_pose, dish_intermediate_pose, dish_up_pose, dish_pull_ready = self.receive_dig_pose(
                    reference_frame)
                rospy.sleep(0.2)
                self.point_sub.unregister()

                self.dig_move_robot(Group, Group_end_effector, dish_start_pose,
                                    dish_intermediate_pose, dish_end_pose, dish_up_pose, reference_frame)


        except:
            pass

    # 计算姿态旋转角
    def convert_rpy(self, position_x, position_y):
        new_position_x = position_x + 0.37
        rotate_z = math.atan2(position_y * 1.0 ,new_position_x)
        return rotate_z


    def move_recognize(self, mqtt_client, total_num, move_one_distance):
        # total_num = 2
        i = total_num
        mqtt_client.publish("/car_steTo_zero", 1)
        dish_location_odom = {}
        while (i):
            # rospy.sleep(3.0)
            scan_dish.car_move(move_one_distance, mqtt_client)
            rospy.sleep(3.0)
            food_name = self.rw_recognize_result(i, total_num)

            i -= 1

        scan_dish.car_move(-total_num * move_one_distance, mqtt_client)
        scan_dish.send_memu()
        return dish_location_odom

    def convert_encode(self, x, y):
        # print depth_image
        rospy.sleep(1.0)
        real_z = 0.001 * self.depth_image[y, x]
        real_x = (x - self.camera_info.K[2]) / self.camera_info.K[0] * real_z
        real_y = (y - self.camera_info.K[5]) / self.camera_info.K[4] * real_z
        return real_x, real_y, real_z

    def rw_recognize_result(self, i, total_num):
        fw = open('/home/robot/photo/recognize_dish_test.txt', 'r+')
        result = eval(fw.read())
        # if(i==0)
        only_food_name = result.keys()[0]

        target_left_x = result[only_food_name][0][0]
        target_left_y = result[only_food_name][0][1]
        # print target_left_y
        camera_left_x, camera_left_y, camera_left_z = self.convert_encode(target_left_x, target_left_y)

        target_right_x = result[only_food_name][0][2]
        target_right_y = result[only_food_name][0][3]
        camera_right_x, camera_right_y, camera_right_z = self.convert_encode(target_right_x, target_right_y)
        fw.close()

        camera_converted = {}
        camera_converted[only_food_name] = [[camera_left_x, camera_left_y, camera_right_x],
                                            [camera_right_x, camera_right_y, camera_right_z], [(total_num - i + 1) * dish_interval]]
        print(camera_converted)
        jsObj = json.dumps(camera_converted)
        if (i == total_num):

            with open("/home/robot/photo/memu_weizhi_test.txt", "wb") as f:
                f.write(str(camera_converted) + '\n')
        else:
            with open("/home/robot/photo/memu_weizhi_test.txt", "ab") as f:

                f.write(str(camera_converted) + '\n')
        return only_food_name


    def bowl_catch(self, Group, Group_endeffector, reference_frame, AG95_client, bowl_num):
        # 读取订单有几种菜品信息来选取碗的数量和位置
        bowl_pose_list_xyz = []
        if bowl_num < 8 and bowl_num > 0:
            bowl_pose_list_xyz = self.catchBowl(bowl_num, Group, Group_endeffector,reference_frame, AG95_client)
        else:
            print("There is no extra place to put the bowl on the plate!\n")
        return bowl_pose_list_xyz

    def catchBowl(self, bowl_num, Group, Group_endeffector, reference_frame, AG95_client):
        bowl_pose_list_xyz = []
        ur3_bowl_pose = PoseStamped()
        ur3_bowl_pose.header.frame_id = reference_frame
        ur3_bowl_pose.header.stamp = rospy.Time.now()
        ur3_bowl_pose.pose.position.x = -0.952546664106
        ur3_bowl_pose.pose.position.y = 0.00982177241731
        ur3_bowl_pose.pose.position.z = 0.872903783818
        ur3_bowl_pose.pose.orientation.x = -0.507184905597
        ur3_bowl_pose.pose.orientation.y = -0.49463697324
        ur3_bowl_pose.pose.orientation.z = 0.491520330079
        ur3_bowl_pose.pose.orientation.w = 0.50646372166

        ur3_wait_pose = PoseStamped()
        ur3_wait_pose.header.frame_id = reference_frame
        ur3_wait_pose.header.stamp = rospy.Time.now()
        ur3_wait_pose.pose.position.x = -0.780578962002
        ur3_wait_pose.pose.position.y = -0.394537337317
        ur3_wait_pose.pose.position.z = 0.860062127016
        ur3_wait_pose.pose.orientation.x = -0.231488483813
        ur3_wait_pose.pose.orientation.y = -0.665534429731
        ur3_wait_pose.pose.orientation.z = 0.670206753147
        ur3_wait_pose.pose.orientation.w = 0.233023416723

        # ur3_dish_pose = PoseStamped()
        ur3_dish_pose.header.frame_id = reference_frame
        ur3_dish_pose.header.stamp = rospy.Time.now()
        ur3_dish_pose.pose.position.x = -0.355989505018
        ur3_dish_pose.pose.position.y = -0.570190369516
        ur3_dish_pose.pose.position.z = 0.759730899083
        ur3_dish_pose.pose.orientation.x = -0.0321627088731
        ur3_dish_pose.pose.orientation.y = -0.741661084261
        ur3_dish_pose.pose.orientation.z = 0.669884629718
        ur3_dish_pose.pose.orientation.w = 0.0126086921934

        TempWaypoint = []
        TempWaypoint.append(ur3_bowl_pose)
        TempWaypoint.append(ur3_wait_pose)

        # 闭合夹爪
        grippers_control.AG95_gripper(100, 0, AG95_client)

        # rospy.sleep(1111111111)
        if bowl_num == 1:
            drop_bowl.send_order_drop_bowl()
            rospy.sleep(1.0)
            Group.set_joint_value_target(UR3_Q2)
            Group.go()
            # print Group.get_current_pose(Group_endeffector)
            Group.set_pose_target(ur3_dish_pose, Group_endeffector)
            Group.go()
            ur3_dish_down_pose = copy.deepcopy(ur3_dish_pose)
            ur3_dish_down_pose.pose.position.z -= 0.05
            Group.set_pose_target(ur3_dish_down_pose, Group_endeffector)
            Group.go()
            grippers_control.AG95_gripper(100, 100, AG95_client)
            Group.set_pose_target(ur3_dish_pose, Group_endeffector)
            Group.go()
            Group.set_joint_value_target(UR3_Q2)
            Group.go()
            Group.set_joint_value_target(UR3_Q1)
            Group.go()
            bowl_pose_list_xyz.append([ur3_dish_pose.pose.position.x, ur3_dish_pose.pose.position.y, ur3_dish_pose.pose.position.z])

            '''
            ur3_dishDown_pose = copy.deepcopy(ur3_dish_pose)
            ur3_dishDown_pose.pose.position.z -= 0.05

            GoToDishWaypoint = TempWaypoint

            GoToDishWaypoint.append(deepcopy(ur3_dish_pose))
            GoToDishWaypoint.append(deepcopy(ur3_dishDown_pose))

            GoToDishPlan = trajectory_plan.B_curve_plan(Group, GoToDishWaypoint)

            drop_bowl.send_order_drop_bowl()
            rospy.sleep(1.0)
            Group.execute(GoToDishPlan)
            # 打开夹爪
            grippers_control.AG95_gripper(100, 100, AG95_client)

            self.UR3BackToBowl(Group, ur3_dishDown_pose, ur3_dish_pose, ur3_wait_pose, ur3_bowl_pose)
            '''

        # elif bowl_num == 2:
        else:

            dish_left_pose = copy.deepcopy(ur3_dish_pose)
            dish_left_pose.pose.position.x -= 0.18

            dish_left_down_pose = copy.deepcopy(dish_left_pose)
            dish_left_down_pose.pose.position.z -= 0.05

            dish_right_pose = copy.deepcopy(ur3_dish_pose)
            dish_right_pose.pose.position.x += 0.01

            dish_right_down_pose = copy.deepcopy(dish_right_pose)
            dish_right_down_pose.pose.position.z -= 0.05

            drop_bowl.send_order_drop_bowl()
            rospy.sleep(1.0)

            Group.set_joint_value_target(UR3_Q2)
            Group.go()
            Group.set_pose_target(dish_left_pose,Group_endeffector)
            Group.go()
            Group.set_pose_target(dish_left_down_pose,Group_endeffector)
            Group.go()
            grippers_control.AG95_gripper(100, 100, AG95_client)
            Group.set_pose_target(dish_left_pose,Group_endeffector)
            Group.go()
            Group.set_joint_value_target(UR3_Q2)
            Group.go()

            Group.set_joint_value_target(UR3_Q1)
            Group.go()
            grippers_control.AG95_gripper(100, 0, AG95_client)
            drop_bowl.send_order_drop_bowl()
            rospy.sleep(1.0)

            Group.set_joint_value_target(UR3_Q2)
            Group.go()
            Group.set_pose_target(dish_right_pose, Group_endeffector)
            Group.go()
            Group.set_pose_target(dish_right_down_pose, Group_endeffector)
            Group.go()
            grippers_control.AG95_gripper(100, 100, AG95_client)
            Group.set_pose_target(dish_right_pose, Group_endeffector)
            Group.go()
            Group.set_joint_value_target(UR3_Q2)
            Group.go()
            Group.set_joint_value_target(UR3_Q1)
            Group.go()

            bowl_pose_list_xyz.append([dish_left_pose.pose.position.x, dish_left_pose.pose.position.y,
                                       dish_left_pose.pose.position.z])
            bowl_pose_list_xyz.append([dish_right_pose.pose.position.x, dish_right_pose.pose.position.y,
                                       dish_right_pose.pose.position.z])

            '''
            Left_DishWaypoint = TempWaypoint
            Left_DishWaypoint.append(dish_left_pose)
            Left_DishWaypoint.append(dish_left_down_pose)

            Right_DishWaypoint = TempWaypoint
            Right_DishWaypoint.append(dish_right_pose)
            Right_DishWaypoint.append(dish_right_down_pose)
            
            drop_bowl.send_order_drop_bowl()
            rospy.sleep(1.0)
            Left_DishPlan = trajectory_plan.B_curve_plan(Group, Left_DishWaypoint)
            print ("First bowl plan is over!")

            Group.execute(Left_DishPlan)
            # 打开夹爪
            grippers_control.AG95_gripper(100, 100, AG95_client)

            self.UR3BackToBowl(Group, dish_left_down_pose, dish_left_pose, ur3_wait_pose, ur3_bowl_pose)
            # 闭合夹爪
            grippers_control.AG95_gripper(100, 0, AG95_client)

            drop_bowl.send_order_drop_bowl()
            rospy.sleep(1.0)

            Right_DishPlan = trajectory_plan.B_curve_plan(Group, Right_DishWaypoint)
            print ("Second bowl plan is over!")

            Group.execute(Right_DishPlan)
            # 打开夹爪
            grippers_control.AG95_gripper(100, 100, AG95_client)
            self.UR3BackToBowl(Group, dish_right_down_pose, dish_right_pose, ur3_wait_pose, ur3_bowl_pose)
            # 闭合夹爪
            grippers_control.AG95_gripper(100, 0, AG95_client)
            '''
        return bowl_pose_list_xyz

    def UR3BackToBowl(self, Group, DishDownPose, DishReadyPose, WaitPose, BowlPose):

        BackToBowlWaypoint = []
        BackToBowlWaypoint.append(deepcopy(DishDownPose))
        BackToBowlWaypoint.append(deepcopy(DishReadyPose))
        BackToBowlWaypoint.append(deepcopy(WaitPose))
        BackToBowlWaypoint.append(deepcopy(BowlPose))

        BackToBowlPlan = trajectory_plan.B_curve_plan(Group, BackToBowlWaypoint)
        Group.execute(BackToBowlPlan)



    def info_callback(self, data):
        self.camera_info = data


    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
            # print self.depth_image
        except CvBridgeError as e:
            print(e)


    def point_callback(self, msg):
        self.original_pointcloud = orh.rospc_to_o3dpc(msg)
        self.original_pointcloud.paint_uniform_color([0.5, 0.5, 0.5])
        self.down_pointcloud = self.original_pointcloud.voxel_down_sample(voxel_size=0.009)
        # o3d.visualization.draw_geometries([self.down_pointcloud])
        self.yuandian = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
        self.rot_dish_area_pointcloud = judge_dish_location.pcd_rot(self.down_pointcloud, 0, 0.599, 0)
        # o3d.visualization.draw_geometries([self.rot_dish_area_pointcloud])
        # self.rot_dish_area_pointcloud.paint_uniform_color([0.5, 0.5, 0.5])
        dish_area_pointcloud = judge_dish_location.cut_safe_area_pointcloud(self.rot_dish_area_pointcloud)
        dish_all_area_pointcloud = judge_dish_location.cut_to_dish_pointcloud(self.rot_dish_area_pointcloud)

        # o3d.visualization.draw_geometries([dish_all_area_pointcloud, self.yuandian])

        judge_dish_location.send_location(dish_area_pointcloud, dish_all_area_pointcloud, self.dish_box_depth_in_camera)
        # self.send_location()

        # return down_pointcloud

    def push_move_robot(self, ur10_arm_spoon, ur10_spoon_end_effector_link, push_start_pose, push_end_pose):
        ur10_arm_spoon.set_pose_target(push_start_pose, ur10_spoon_end_effector_link)
        rospy.sleep(0.2)
        ur10_arm_spoon.go()
        # waypoint = []
        # waypoint.append(push_start_pose)
        #
        # waypoint.append(push_end_pose)
        #
        # planUR10 = self.B_curve_plan(ur10_arm_spoon, waypoint)
        # ur10_arm_spoon.execute(planUR10)

        ur10_arm_spoon.set_pose_target(push_end_pose, ur10_spoon_end_effector_link)
        rospy.sleep(0.2)
        ur10_arm_spoon.go()

        ur10_arm_spoon.set_joint_value_target(UR10_Q1)
        ur10_arm_spoon.go()

    def dig_move_robot(self, ur10_arm_spoon, ur10_spoon_end_effector_link, dish_start_pose, dish_intermediate_pose, dish_end_pose, dish_up_pose, reference_frame):
        ur10_arm_spoon.set_pose_target(dish_start_pose, ur10_spoon_end_effector_link)
        move_success = ur10_arm_spoon.go()
        if move_success == False:
            print "go_back"
            self.catch_dish(ur10_arm_spoon,ur10_spoon_end_effector_link, reference_frame)


        waypoint = []

        waypoint.append(dish_start_pose)

        waypoint.append(deepcopy(dish_intermediate_pose))

        waypoint.append(deepcopy(dish_end_pose))

        # waypoint.append(deepcopy(dish_up_pose))

        planUR10 = trajectory_plan.B_curve_plan(ur10_arm_spoon, waypoint)
        if planUR10 == None:
            print "go_UR_Q1"
            ur10_arm_spoon.set_joint_value_target(UR10_Q1)
            go_initial = ur10_arm_spoon.go()
            if go_initial == True:
                self.catch_dish(ur10_arm_spoon, ur10_spoon_end_effector_link, reference_frame)
            else:
                moveit_commander.roscpp_shutdown()
                moveit_commander.os._exit(0)
        ur10_arm_spoon.execute(planUR10)

        # ur10_arm_spoon.set_pose_target(dish_up_pose, ur10_spoon_end_effector_link)
        # ur10_arm_spoon.go()



    def receive_push_pose(self, reference_frame):
        read_push_start = tf.TransformListener()
        read_push_end = tf.TransformListener()

        read_push_start.waitForTransform("/body_link", "/push_start", rospy.Time(), rospy.Duration(1.0))
        read_push_end.waitForTransform("/body_link", "/push_end", rospy.Time(), rospy.Duration(1.0))

        try:
            push_start_pose = PoseStamped()
            push_end_pose = PoseStamped()

            (push_start_trans, push_start_rot) = read_push_start.lookupTransform("/body_link", "/push_start", rospy.Time(0))
            push_start_pose.header.frame_id = reference_frame
            push_start_pose.header.stamp = rospy.Time(0)
            push_start_pose.pose.position.x = push_start_trans[0]
            push_start_pose.pose.position.y = push_start_trans[1]
            push_start_pose.pose.position.z = push_start_trans[2]

            push_start_pose.pose.orientation.x = push_start_rot[0]
            push_start_pose.pose.orientation.y = push_start_rot[1]
            push_start_pose.pose.orientation.z = push_start_rot[2]
            push_start_pose.pose.orientation.w = push_start_rot[3]

            (push_end_trans, push_end_rot) = read_push_end.lookupTransform("/body_link", "/push_end",
                                                                                 rospy.Time(0))
            push_end_pose.header.frame_id = reference_frame
            push_end_pose.header.stamp = rospy.Time(0)
            push_end_pose.pose.position.x = push_end_trans[0]
            push_end_pose.pose.position.y = push_end_trans[1]
            push_end_pose.pose.position.z = push_end_trans[2]

            push_end_pose.pose.orientation.x = push_end_rot[0]
            push_end_pose.pose.orientation.y = push_end_rot[1]
            push_end_pose.pose.orientation.z = push_end_rot[2]
            push_end_pose.pose.orientation.w = push_end_rot[3]

        except:
            pass

        return push_start_pose, push_end_pose

    def receive_dig_pose(self, reference_frame):
        read_start_rot = tf.TransformListener()
        read_end_rot = tf.TransformListener()
        read_intermediate_rot = tf.TransformListener()
        # rospy.sleep(0.2)

        # get_location.transform_to_origin()
        read_start_rot.waitForTransform("/body_link", "/start_loca", rospy.Time(), rospy.Duration(1.0))

        read_intermediate_rot.waitForTransform("/body_link", "/intermediate_loca", rospy.Time(), rospy.Duration(1.0))

        read_end_rot.waitForTransform("/body_link", "/end_loca", rospy.Time(), rospy.Duration(1.0))
        # print 1
        try:

            (start_trans, start_rot) = read_start_rot.lookupTransform("/body_link", "/start_loca", rospy.Time(0))

            spoon_start_pose = PoseStamped()
            spoon_intermediate_pose = PoseStamped()
            spoon_end_pose = PoseStamped()
            spoon_up_pose = PoseStamped()
            spoon_pull_ready = PoseStamped()

            spoon_start_pose.header.frame_id = reference_frame
            spoon_start_pose.header.stamp = rospy.Time(0)
            spoon_start_pose.pose.position.x = start_trans[0]
            spoon_start_pose.pose.position.y = start_trans[1]
            spoon_start_pose.pose.position.z = start_trans[2]

            spoon_start_pose.pose.orientation.x = start_rot[0]
            spoon_start_pose.pose.orientation.y = start_rot[1]
            spoon_start_pose.pose.orientation.z = start_rot[2]
            spoon_start_pose.pose.orientation.w = start_rot[3]
            rospy.sleep(0.2)
            print("start_pose")
            print spoon_start_pose.pose.position
            (intermediate_trans, intermediate_rot) = read_intermediate_rot.lookupTransform("/body_link", "/intermediate_loca", rospy.Time(0))

            spoon_intermediate_pose.header.frame_id = reference_frame
            spoon_intermediate_pose.header.stamp = rospy.Time(0)
            spoon_intermediate_pose.pose.position.x = intermediate_trans[0]
            spoon_intermediate_pose.pose.position.y = intermediate_trans[1]
            spoon_intermediate_pose.pose.position.z = intermediate_trans[2]

            spoon_intermediate_pose.pose.orientation.x = start_rot[0]
            spoon_intermediate_pose.pose.orientation.y = start_rot[1]
            spoon_intermediate_pose.pose.orientation.z = start_rot[2]
            spoon_intermediate_pose.pose.orientation.w = start_rot[3]

            rospy.sleep(0.2)
            (end_trans, end_rot) = read_end_rot.lookupTransform("/body_link", "/end_loca", rospy.Time(0))
            spoon_end_pose.header.frame_id = reference_frame
            spoon_end_pose.header.stamp = rospy.Time(0)
            spoon_end_pose.pose.position.x = end_trans[0]
            spoon_end_pose.pose.position.y = end_trans[1]
            spoon_end_pose.pose.position.z = end_trans[2]

            spoon_end_pose.pose.orientation.x = -0.0242689917093
            spoon_end_pose.pose.orientation.y = -0.000871234937564
            spoon_end_pose.pose.orientation.z = -0.0446856539918
            spoon_end_pose.pose.orientation.w = 0.998705887295
            # print "end_pose"
            # print spoon_end_pose.pose.position

            spoon_up_pose.header.frame_id = reference_frame
            spoon_up_pose.header.stamp = rospy.Time(0)
            spoon_up_pose.pose.position.x = end_trans[0]
            spoon_up_pose.pose.position.y = end_trans[1]
            spoon_up_pose.pose.position.z = end_trans[2] + 0.1

            spoon_up_pose.pose.orientation.x = -0.0242689917093
            spoon_up_pose.pose.orientation.y = -0.000871234937564
            spoon_up_pose.pose.orientation.z = -0.0446856539918
            spoon_up_pose.pose.orientation.w = 0.998705887295

            spoon_pull_ready.header.frame_id = reference_frame
            spoon_pull_ready.header.stamp = rospy.Time(0)
            spoon_pull_ready.pose.position.x = ur3_dish_pose.pose.position.x
            spoon_pull_ready.pose.position.y = ur3_dish_pose.pose.position.y
            spoon_pull_ready.pose.position.z = ur3_dish_pose.pose.position.z

            spoon_pull_ready.pose.orientation.x = -0.0242689917093
            spoon_pull_ready.pose.orientation.y = -0.000871234937564
            spoon_pull_ready.pose.orientation.z = -0.0446856539918
            spoon_pull_ready.pose.orientation.w = 0.998705887295

            rospy.sleep(0.2)

        ##########################################################################
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        return spoon_start_pose,  spoon_end_pose, spoon_intermediate_pose, spoon_up_pose, spoon_pull_ready



if __name__ == '__main__':


    down_pointcloud = o3d.geometry.PointCloud()
    try:
        Get_location()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass