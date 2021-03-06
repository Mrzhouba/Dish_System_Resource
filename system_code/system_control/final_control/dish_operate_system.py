#!/usr/bin/env python
#coding=UTF-8
import sys, subprocess, rospy, time
import serial, binascii
import paho.mqtt.client as mqtt
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import qdarkstyle
import multiprocessing, psutil
from playsound import playsound
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

from sensor_msgs.msg import CompressedImage,Image, JointState
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
import cv2
import requests, json
import roslaunch
from robot_order.msg import food_msg
import cv2
import cv2.aruco as aruco




sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/assist_modules')
import Variable
from serial_drop_bowl import drop_bowl
drop_bowl_instance = drop_bowl()


from scan_dish import scan_dish
scan_dish = scan_dish()
from dual_arm_reset import *
UR3_reset = UR3_reset()
UR10_reset = UR10_reset()

sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/pyqt5_test/Wrench_Show')
from Wrench_plot import Wrench_plot_Page

sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish')
from yolo import YOLO
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/pyqt5_test/final_control/dual_arm_dig_dish')
from final_dig_qt import final_ur3ur10_alone_work_qt
# final_ur3ur10_alone_work_qt = final_ur3ur10_alone_work_qt()

from add_bowl_action import *
ur3_add_bowl_action = ur3_add_bowl_action()

from Initial_work import Ui_Initial_Form
from generate_menu import Ui_Menu_Form
from dig_dish import Ui_Dig_dish_Form
from all_window import Ui_All_MainWindow

HOST = Variable.HOST
PORT = Variable.PORT
Initial_loc = -0.048675943166

pause_usb_name = Variable.pause_usb_name

class Initial_Work_Page(QMainWindow, Ui_Initial_Form):
    def __init__(self, Parent=None):
        super(Initial_Work_Page, self).__init__(Parent)
        self.timer = QTimer()
        self.timer.timeout.connect(self.showtime)
        self.timer.start(1000)

        self.setupUi(self)
        # self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint | Qt.Tool)
        self.setLed_size_color()
        self.setbutton_status()

        self.system_monitor_thread = monitor_system_status()

        self.initial_thread = initial_Worker()
        self.close_thread = close_Worker()
        self.power_monitor_thread = monitor_power_status()
        self.emergency_stop_thread = emergency_stop_Worker()
        self.recognize_start_thread = recognize_start_loc()
        self.wrench_show_thread = WrenchPlot_Worker()

        temp_time = QDateTime.currentDateTime()
        self.timedisplay = temp_time.toString("hh:mm:ss")
        self.save_initial_txt_path = []
        self.CreateSignalSlot()

        self.initial_pic = QPixmap('./kangli.png')
        self.aruco_label.setPixmap(self.initial_pic)
        self.aruco_label.setScaledContents(True)



    def setLed_size_color(self):
        self.Led.setMinimumSize(80, 80)
        self.power_led.setMinimumSize(45,45)
        self.power_led.setLedOption('colorOnBegin', QColor(255, 255, 0))
        self.power_led.setLedOption('colorOnEnd', QColor(255, 200, 0))
        self.power_led.setLedOption('colorOffBegin', QColor(150, 150, 0))
        self.power_led.setLedOption('colorOffEnd', QColor(100, 100, 0))

    def setbutton_status(self):
        self.car_to_zero_button.setEnabled(False)
        self.car_forward.setEnabled(False)
        self.car_back.setEnabled(False)
        self.convey_forward.setEnabled(False)
        self.convey_back.setEnabled(False)
        self.sys_end.setEnabled(False)

        self.UR3_to_zero.setEnabled(False)
        self.UR10_to_zero.setEnabled(False)

    def CreateSignalSlot(self):
        # self.system_monitor_thread.create_rosnode()
        self.camera_image = CameraRosNode()
        self.camera_image.aruco_result_signal.connect(self.show_aruco_result)

        self.sys_start.clicked.connect(self.start_initial_thread)

        self.sys_end.clicked.connect(self.start_close_thread)

        self.power_monitor_thread.power_change.connect(self.power_status_switch)
        self.power_monitor_thread.mqtt_server_fail.connect(self.mqtt_connect_warning)
        # ??????????????????
        self.system_monitor_thread.system_status_change.connect(self.system_status_switch)

        self.power_switch.clicked.connect(self.car_power_control)
        self.emergency_stop_thread.emeregency_stop_signal.connect(self.emergency_stop_control)
        self.emergency_stop_thread.diconnect_ser_signal.connect(self.switch_disconnect_warning)
        self.emergency_stop_thread.emergency_button_signal.connect(self.emergency_button_status)
        self.wrench_show_thread.wrench_show_status_signal.connect(self.show_wrenchPlot_status)

        self.drop_bowl.clicked.connect(self.drop_bowl_control)
        self.clear_initial_txt.clicked.connect(self.clear_initial_message)

        self.recognize_start_thread.need_move_dis.connect(self.show_need_moveDis)
        self.recognize_mark_button.clicked.connect(self.start_label)
        self.car_need_moveDis = 0
        self.car_to_zero_button.clicked.connect(self.car_move_startLoc)

        #dual arm reset
        self.UR10_to_zero.clicked.connect(self.ur10_move_zero)
        self.UR3_to_zero.clicked.connect(self.ur3_move_zero)

        self.save_initial_txt.clicked.connect(self.save_initial_log)

        self.emergency_stop_thread.start()
        self.power_monitor_thread.start()


    def show_wrenchPlot_status(self, msg):
        if msg == 1:
            self.Initial_messgae_txt.append(str(self.timedisplay) + ": ??????????????????")


    def save_initial_log(self):
        if len(self.save_initial_txt_path) == 0:
            self.save_initial_txt_path = QFileDialog.getSaveFileName(self, "????????????","/home/robot/photo","All Files (*);;Text Files (*.txt)")
            self.Initial_messgae_txt.append('?????????????????????' + str(self.save_initial_txt_path[0]))
        else:
            # print self.save_initial_txt_path
            initial_log = self.Initial_messgae_txt.toPlainText()
            f = open(self.save_initial_txt_path[0], 'w')
            f.write(initial_log)
            f.close()

    def ur3_move_zero(self):
        UR3_reset.ur3_reset_work()

        self.Initial_messgae_txt.append(str(self.timedisplay) + ': ur3??????????????????')

    def ur10_move_zero(self):
        UR10_reset.ur10_reset_work()

        self.Initial_messgae_txt.append(str(self.timedisplay) + ': ur10??????????????????')


    def car_move_startLoc(self):
        car_move_client = mqtt.Client("car-control")
        car_move_client.connect(HOST, PORT, 60)
        car_move_client.loop_start()

        scan_dish.car_move(self.car_need_moveDis, car_move_client)
        self.car_to_zero_button.setEnabled(False)
        self.car_to_zero_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset')
        # self.recognize_mark_button.setEnabled(True)
        # self.recognize_mark_button.setStyleSheet('color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.Initial_messgae_txt.append(str(self.timedisplay) + ": ??????????????????")

    def showtime(self):
        time=QDateTime.currentDateTime()
        self.timedisplay=time.toString("hh:mm:ss")
        # print type(str(self.timedisplay))

    def show_need_moveDis(self, msg):
        if msg == 0:
            self.Initial_messgae_txt.append(str(self.timedisplay) + ': ??????????????????')
        else:
            self.car_need_moveDis = msg
            self.Initial_messgae_txt.append(str(self.timedisplay) + ': ??????????????????{}m'.format(self.car_need_moveDis))
            if self.power_switch.text() == 'power_off':
                self.car_to_zero_button.setEnabled(True)
                self.car_to_zero_button.setStyleSheet(
                    'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
                # self.recognize_mark_button.setEnabled(False)
                # self.recognize_mark_button.setStyleSheet(
                #     'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            else:
                QMessageBox.information(self, "??????", "???????????????", QMessageBox.Yes)

    def start_label(self):
        self.recognize_start_thread.start()
        # try:
        #     data = rospy.wait_for_message('/aruco_single/position', Vector3Stamped, timeout=1)
        #     self.Initial_messgae_txt.append(str(self.timedisplay) + ': ??????????????????')
        #     self.car_need_moveDis = data.vector.y - Initial_loc
        # except:
        #     self.Initial_messgae_txt.append(str(self.timedisplay) + ': ??????????????????')
        #     self.car_need_moveDis = 0
        # if self.car_need_moveDis != 0:
        #     self.Initial_messgae_txt.append(str(self.timedisplay) + ': ??????????????????{}m'.format(self.car_need_moveDis))
        #     if self.power_switch.text() == 'power_off':
        #         self.car_to_zero_button.setEnabled(True)
        #         self.car_to_zero_button.setStyleSheet('color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
    def car_power_control(self):
        power_client = mqtt.Client("ubuntu-carpower-control")
        power_client.connect(HOST, PORT, 60)
        if self.power_switch.text() == "power_on":
            power_client.publish("/control_power", "car_pos,1")
            time.sleep(0.1)
            self.emergency_stop_thread.start()
        else:
            power_client.publish("/control_power", "car_pos,0")
            time.sleep(0.1)


    def emergency_stop_control(self):
        self.Initial_messgae_txt.append(str(self.timedisplay) + ": ????????????")

        self.camera_image.end_process = True
        self.system_monitor_thread.end_process = True

        self.emergency_power_off()
        self.start_close_thread()

    def switch_disconnect_warning(self):
        QMessageBox.warning(self, "??????", "?????????????????????", QMessageBox.Yes)
        sys.exit(0)

    def emergency_button_status(self, msg):
        if msg == 1:
            self.initial_button_disable(False)
            reply = QMessageBox.warning(self, "??????", "?????????????????????", QMessageBox.Yes)
            if reply == QMessageBox.Yes:
                self.emergency_stop_thread.messageBox_close()
        elif msg == 0:
            self.initial_button_disable(True)

    def initial_button_disable(self, status):
        if status == False:
            self.sys_start.setEnabled(False)
            self.sys_start.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.power_switch.setEnabled(False)
            self.power_switch.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.drop_bowl.setEnabled(False)
            self.drop_bowl.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        elif status == True:
            self.sys_start.setEnabled(True)
            self.sys_start.setStyleSheet('color:rgb(255,96,96);border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.power_switch.setEnabled(True)
            self.power_switch.setStyleSheet('color:rgb(178,34,34);border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.drop_bowl.setEnabled(True)
            self.drop_bowl.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')


    def mqtt_connect_warning(self):
        QMessageBox.information(self, "??????", "mqtt?????????????????????", QMessageBox.Yes)
        sys.exit(0)

    def drop_bowl_control(self):
        bowl_client = mqtt.Client("ubuntu-bowl-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.loop_start()
        is_drop = drop_bowl_instance.send_order_drop_bowl(bowl_client)
        if is_drop == True:
            self.Initial_messgae_txt.append(str(self.timedisplay) + ": ????????????")
            bowl_client.disconnect()
        else:
            pass

    def emergency_power_off(self):
        emergency_client = mqtt.Client("ubuntu-carpower-control")
        emergency_client.connect(HOST, PORT, 60)
        emergency_client.publish("/control_power", "car_pos,0")
        time.sleep(0.1)

    def car_power_on(self):
        emergency_client = mqtt.Client("ubuntu-carpower-control")
        emergency_client.connect(HOST, PORT, 60)
        emergency_client.publish("/control_power", "car_pos,1")
        time.sleep(0.1)

    def start_launch(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        robot_launch = roslaunch.parent.ROSLaunchParent(uuid, [
            "/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/launch"
            "/dual_arm_car_grippers_fts_metal_publish_alone.launch"])
        camera_launch = roslaunch.parent.ROSLaunchParent(uuid, [
            "/home/robot/catkin_ws/src/realsense-ros-development/realsense2_camera/launch/rs_camera_vegetable.launch"])
        hand_launch = roslaunch.parent.ROSLaunchParent(uuid, [
            "/home/robot/catkin_ws/src/dh_hand_driver/launch/dh_hand_controller.launch"])


        robot_launch.start()
        rospy.sleep(0.1)
        camera_launch.start()
        rospy.sleep(0.1)
        hand_launch.start()
        rospy.sleep(0.1)

    def start_initial_thread(self):
        self.camera_image.end_process = False
        self.system_monitor_thread.end_process = False
        # self.initial_thread.start()
        self.start_launch()
        self.system_monitor_thread.start()
        self.camera_image.start()
        if self.start_with_car_power_button.isChecked():
            self.car_power_on()

    def start_close_thread(self):
        self.emergency_power_off()
        self.close_thread.start()

        self.camera_image.end_process = True
        self.system_monitor_thread.end_process = True

        self.UR3_to_zero.setEnabled(False)
        self.UR3_to_zero.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.UR10_to_zero.setEnabled(False)
        self.UR10_to_zero.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.recognize_mark_button.setEnabled(False)
        self.recognize_mark_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')


        # self.system_monitor_thread.quit()
        # print self.system_monitor_thread.isRunning()
        # print self.system_monitor_thread.isFinished()
        # self.dig_dish_thread.quit()
        # self.Dish_Task.setEnabled(True)
        # print self.dig_dish_thread.isFinished()
        # print self.dig_dish_thread.isRunning()

    def show_aruco_result(self, msg):
        label_width = self.aruco_label.width()
        label_height = self.aruco_label.height()
        temp_imgSrc = QImage(msg, msg.shape[1], msg.shape[0], msg.shape[1] * 3, QImage.Format_RGB888)
        pixmap_imgSrc = QPixmap.fromImage(temp_imgSrc).scaled(label_width, label_height)

        self.aruco_label.setPixmap(pixmap_imgSrc)

    def power_status_switch(self, msg):
        if msg == 0:
            self.power_led.setEnabled(False)
            self.power_switch.setText("power_on")
            self.car_back.setEnabled(False)
            self.car_back.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.car_forward.setEnabled(False)
            self.car_forward.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.convey_forward.setEnabled(False)
            self.convey_forward.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.convey_back.setEnabled(False)
            self.convey_back.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.Initial_messgae_txt.append(str(self.timedisplay) + ": ????????????")
        if msg == 1:
            self.power_led.setEnabled(True)
            self.power_switch.setText("power_off")
            self.car_back.setEnabled(True)
            self.car_back.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.car_forward.setEnabled(True)
            self.car_forward.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.convey_forward.setEnabled(True)
            self.convey_forward.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.convey_back.setEnabled(True)
            self.convey_back.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.Initial_messgae_txt.append(str(self.timedisplay) + ": ????????????")

    def system_status_switch(self, msg):
        if msg == 0:
            self.Led.setEnabled(False)

            self.sys_start.setEnabled(True)
            self.sys_start.setStyleSheet(
                "color:rgb(255,96,96);border-radius: 10px; border: 2px groove gray;border-style: outset;")
            self.sys_end.setEnabled(False)
            self.sys_end.setStyleSheet("color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;")

            self.Initial_messgae_txt.append(str(self.timedisplay) + ": ????????????")
            self.aruco_label.setPixmap(self.initial_pic)
        if msg == 1:
            self.Led.setEnabled(True)

            self.sys_start.setEnabled(False)
            self.sys_start.setStyleSheet(
                "color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;")
            self.sys_end.setEnabled(True)
            self.sys_end.setStyleSheet(
                "color:rgb(255,96,96);border-radius: 10px; border: 2px groove gray;border-style: outset;")

            self.Initial_messgae_txt.append(str(self.timedisplay) + ": ????????????")
            self.UR3_to_zero.setEnabled(True)
            self.UR3_to_zero.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.UR10_to_zero.setEnabled(True)
            self.UR10_to_zero.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.recognize_mark_button.setEnabled(True)
            self.recognize_mark_button.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')

    def clear_initial_message(self):
        self.Initial_messgae_txt.clear()

class Generate_Menu_Page(QMainWindow, Ui_Menu_Form):
    def __init__(self, system_monitor_thread, camera_thread):
        super(Generate_Menu_Page, self).__init__()
        self.timer = QTimer()
        self.timer.timeout.connect(self.showtime)
        self.timer.start(1000)
        self.bridge = CvBridge()

        self.setupUi(self)
        self.set_all_control_disable()
        temp_time = QDateTime.currentDateTime()
        self.timedisplay = temp_time.toString("hh:mm:ss")

        self.move_frequency_num = 0
        self.move_interval_num = 0
        self.dish_location = -1
        self.auto_menu = {}
        self.manual_menu = {}
        self.final_menu = {}


        self.yolo_detect = YOLO()
        self.menu_message_text.append(str(self.timedisplay) + ': Loading weights Finished')

        self.system_status_thread_copy = system_monitor_thread

        # ???yolo??????
        self.auto_generate_menu_yolo_thread = auto_gen_menu()

        # aruco??????
        self.auto_generate_menu_label_thread = auto_gen_menu_with_aruco()

        # ??? ????????????
        # self.auto_generate_number_menu_thread = auto_gen_number_menu()

        self.camera_image_copy = camera_thread

        self.CreateSignalSlot()

        self.initial_pic = QPixmap('./kangli.png')
        self.initial_pic_1 = QPixmap('./xiaohui.jpeg')
        self.menu_label.setPixmap(self.initial_pic)
        self.menu_label.setScaledContents(True)

        self.dish_reco_result_label.setPixmap(self.initial_pic_1)
        self.dish_reco_result_label.setScaledContents(True)

        self.save_menu_path = []


    def CreateSignalSlot(self):
        self.system_status_thread_copy.system_status_change.connect(self.change_all_control_enable)
        # self.system_status_thread.start()

        self.camera_image_copy.aruco_result_signal.connect(self.show_raw_image)

        self.auto_mode.clicked.connect(lambda: self.choose_generae_mode(self.auto_mode))
        self.manual_mode.clicked.connect(lambda: self.choose_generae_mode(self.manual_mode))

        self.clear_menu_txt.clicked.connect(self.clear_all_message)

        self.reco_dish_button.clicked.connect(self.manual_reco_dish)
        self.start_scan_button.clicked.connect(self.auto_yolo_reco_dish)
        self.start_scan_label_button.clicked.connect(self.auto_label_reco_dish)
        self.reco_number_button.clicked.connect(self.recognize_number)
        # self.scan_number_button.clicked.connect(self.auto_reco_number)

        self.move_interval_input.textChanged.connect(self.move_interval)
        self.move_frequency_input.textChanged.connect(self.move_frequency)
        self.dish_location_input.textChanged.connect(self.manual_change_loca)

        self.auto_generate_menu_label_thread.food_name_signal.connect(self.deal_auto_reco_result)
        self.auto_generate_menu_label_thread.dish_location_odom_signal.connect(self.deal_auto_reco_result)
        self.auto_generate_menu_label_thread.reco_dish_result.connect(self.deal_auto_reco_result)

        self.auto_generate_menu_yolo_thread.food_name_signal.connect(self.deal_auto_reco_result)
        self.auto_generate_menu_yolo_thread.dish_location_odom_signal.connect(self.deal_auto_reco_result)
        self.auto_generate_menu_yolo_thread.reco_dish_result.connect(self.deal_auto_reco_result)

        # self.auto_generate_number_menu_thread.food_name_number_signal.connect(self.deal_auto_reco_result)
        # self.auto_generate_number_menu_thread.dish_location_odom_number_signal.connect(self.deal_auto_reco_result)
        # self.auto_generate_number_menu_thread.reco_dish_number_result.connect(self.deal_auto_reco_result)

        self.generate_menu_button.clicked.connect(self.generate_menu_work)
        self.save_menu_button.clicked.connect(self.save_menu_work)
        self.upload_menu_button.clicked.connect(self.upload_menu_work)

        self.save_menu_path_button.clicked.connect(self.choose_save_menu_path)


    def recognize_number(self):
        if self.dish_location >= 0:
            food_name = self.pre_num_result()
            if food_name != "None":
                self.menu_message_text.append(str(self.timedisplay) + ' ??????: ' + str(food_name))
                self.manual_menu[food_name] = self.dish_location
            else:
                self.menu_message_text.append(str(self.timedisplay) + ' ??????: ??????????????????')
        else:
            QMessageBox.information(self, "??????", "????????????????????????", QMessageBox.Ok)
        print "end"

    # def auto_reco_number(self):
    #     if 0 < self.move_interval_num < 0.5 and 0 < self.move_frequency_num < 6:
    #         auto_number_client = mqtt.Client("generate_menu")
    #         auto_number_client.connect(HOST, PORT, 60)
    #         auto_number_client.loop_start()
    #
    #         self.menu_message_text.append(
    #             str(self.timedisplay) + '????????? ?????? ' + str(self.move_interval_num) + ', ?????? ' + str(self.move_frequency_num))
    #         i = self.move_frequency_num
    #         total_num = self.move_frequency_num
    #         move_one_distance = self.move_interval_num
    #
    #         while(i):
    #             food_name, _ = self.pre_num_result(self.current_image_np)
    #             self.menu_message_text.append(str(self.timedisplay) + ' ??????: ' + str(food_name))
    #             self.auto_menu[food_name] = move_one_distance * (total_num - i)
    #             if i != 1:
    #                 scan_dish.car_move(move_one_distance, auto_number_client)
    #             i -= 1
    #         rospy.sleep(0.2)
    #         scan_dish.car_move(-(total_num - 1) * move_one_distance, auto_number_client)
    #         rospy.sleep(0.5)
    #         auto_number_client.disconnect()
    #         # self.auto_generate_number_menu_thread.start()
    #         # self.auto_generate_number_menu_thread.get_values(self.move_interval_num, self.move_frequency_num, self.label_detect)
    #
    #     else:
    #         QMessageBox.information(self, "??????", "?????????????????????", QMessageBox.Yes)

    def pre_num_result(self, ):
        aruco_id_to_food = {587: 'LaZiJi', 583: 'JiMiHua', 584: 'ChaoPuGua', 585: 'ZhaTuDou',
                            586: 'RouPiHuangDou'}
        image = self.current_image_np
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        print ids

        food_name = None
        if ids != None:
            food_name = aruco_id_to_food[int(ids[0])]
        return food_name

    def choose_save_menu_path(self):
        path = QFileDialog.getSaveFileName(self, "????????????","/home/robot","All Files (*);;Text Files (*.txt)")
        self.save_menu_path = path[0]

    def generate_menu_work(self):

        if self.auto_menu or self.manual_menu:
            if self.auto_mode.isChecked() == True:
                self.final_menu = self.auto_menu
                self.menu_message_text.append(str(self.timedisplay) + ': ????????????????????????' + str(self.final_menu))
            else:
                self.final_menu = self.manual_menu
                self.menu_message_text.append(str(self.timedisplay) + ': ????????????????????????' + str(self.final_menu))

        else:
                QMessageBox.information(self, "??????", "??????????????????", QMessageBox.Ok)

    def save_menu_work(self):
        if self.final_menu:
            with open("/home/robot/photo/memu_weizhi_test.txt", "wb") as f:
                f.write(str(self.final_menu) + '\n')
            self.menu_message_text.append(str(self.timedisplay) + ": ???????????????????????????")
            self.final_menu.clear()
            self.auto_menu.clear()
            self.manual_menu.clear()
        else:
                QMessageBox.information(self, "??????", "??????????????????", QMessageBox.Ok)

    def upload_menu_work(self):
        fw = open("/home/robot/photo/memu_weizhi_test.txt", "r")
        content = fw.read()
        result = eval(content)
        if result:
            memu = {}
            memu_index = 1
            for key in result:
                memu[str(memu_index)] = key
                memu_index = memu_index + 1
            url = "http://canteen.cailh.club/sell/robot/updatefood?canteen=0&port=1"
            res = requests.post(url=url, data=json.dumps(memu))
            fw.close()
            self.menu_message_text.append(str(self.timedisplay) + ": ???????????????????????????")

        else:
            QMessageBox.information(self, "??????", "????????????,????????????", QMessageBox.Ok)

    def manual_reco_dish(self):
        if self.dish_location >= 0:
            # data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=0.5)
            # frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # frame = self.current_image_np
            image = self.current_image_np
            food_name, image_result = self.yolo_detect.detect_image(image)
            print food_name
            self.show_image(image_result)
            if food_name != "None":
                self.menu_message_text.append(str(self.timedisplay) + ': ' + str(food_name))
                self.manual_menu[food_name] = self.dish_location
            else:
                self.menu_message_text.append(str(self.timedisplay) + ': ??????????????????')
        else:
            QMessageBox.information(self, "??????", "????????????????????????", QMessageBox.Ok)

    def auto_label_reco_dish(self):
        if 0 < self.move_interval_num < 0.5 and 0 < self.move_frequency_num < 6:
            # playsound('/home/robot/sound/scan.mp3', block=True)

            self.menu_message_text.append(
                str(self.timedisplay) + '????????? ' + str(self.move_interval_num) + ', ?????? ' + str(self.move_frequency_num))

            self.auto_generate_menu_label_thread.start()
            self.auto_generate_menu_label_thread.get_values(self.move_interval_num, self.move_frequency_num)
        else:
            QMessageBox.information(self, "??????", "?????????????????????", QMessageBox.Yes)

    def auto_yolo_reco_dish(self):
        if 0 < self.move_interval_num < 0.5 and 0 < self.move_frequency_num < 6:
            # playsound('/home/robot/sound/scan.mp3', block=True)
            self.menu_message_text.append(
                str(self.timedisplay) + '????????? ' + str(self.move_interval_num) + ', ?????? ' + str(self.move_frequency_num))

            self.auto_generate_menu_yolo_thread.start()
            self.auto_generate_menu_yolo_thread.get_values(self.move_interval_num, self.move_frequency_num, self.yolo_detect)

        else:
            QMessageBox.information(self, "??????", "?????????????????????", QMessageBox.Yes)

    def deal_auto_reco_result(self, msg):
        if type(msg) == unicode:
            if str(msg) == "None":
                self.menu_message_text.append(str(self.timedisplay) + ': ??????????????????')
            else:
                self.menu_message_text.append(str(self.timedisplay) + ': ?????????' + str(msg))

        if type(msg) == dict:
            self.menu_message_text.append(str(self.timedisplay) + ': ' + str(msg))
            self.auto_menu = msg
        if type(msg) == np.ndarray:
            self.show_image(msg)

    def move_interval(self):
        current_interval = self.move_interval_input.text()
        if len(str(current_interval)) != 0:
            self.move_interval_num = float(current_interval)

    def move_frequency(self):
        current_frequency = self.move_frequency_input.text()
        if len(str(current_frequency)) != 0:
            self.move_frequency_num = int(current_frequency)

    def manual_change_loca(self):
        current_dish_loc = self.dish_location_input.text()
        if len(str(current_dish_loc)) != 0:
            self.dish_location = float(current_dish_loc)

    def showtime(self):
        time = QDateTime.currentDateTime()
        self.timedisplay = time.toString("hh:mm:ss")

    def show_image(self, msg):
        label_width = self.dish_reco_result_label.width()
        label_height = self.dish_reco_result_label.height()
        temp_imgSrc = QImage(msg, msg.shape[1], msg.shape[0], msg.shape[1] * 3, QImage.Format_RGB888)
        pixmap_imgSrc = QPixmap.fromImage(temp_imgSrc).scaled(label_width, label_height)

        self.dish_reco_result_label.setPixmap(pixmap_imgSrc)

    def show_raw_image(self, msg):
        self.current_image_np = msg
        label_width = self.menu_label.width()
        label_height = self.menu_label.height()
        temp_imgSrc = QImage(msg, msg.shape[1], msg.shape[0], msg.shape[1] * 3, QImage.Format_RGB888)
        pixmap_imgSrc = QPixmap.fromImage(temp_imgSrc).scaled(label_width, label_height)

        self.menu_label.setPixmap(pixmap_imgSrc)

    def clear_all_message(self):
        self.menu_message_text.clear()

    def set_all_control_disable(self):
        self.auto_mode.setChecked(False)
        self.manual_mode.setChecked(False)
        self.car_back_copy.setEnabled(False)
        self.car_forward_copy.setEnabled(False)
        self.auto_mode.setEnabled(False)
        self.manual_mode.setEnabled(False)
        self.reco_dish_button.setEnabled(False)
        self.start_scan_button.setEnabled(False)
        self.start_scan_label_button.setEnabled(False)
        self.generate_menu_button.setEnabled(False)
        self.save_menu_button.setEnabled(False)
        self.upload_menu_button.setEnabled(False)
        self.move_frequency_input.setEnabled(False)
        self.move_interval_input.setEnabled(False)
        self.dish_location_input.setEnabled(False)
        self.save_menu_path_button.setEnabled(False)
        self.reco_number_button.setEnabled(False)

        self.car_back_copy.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.car_forward_copy.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.reco_dish_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.start_scan_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.start_scan_label_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.generate_menu_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.save_menu_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.upload_menu_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.reco_number_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')

    def choose_generae_mode(self, btn):
        if btn.text() == "????????????".decode('utf-8'):
            if btn.isChecked() == True:
                self.auto_mode.setChecked(False)
                self.change_mode_control_enable('manual')
                self.auto_menu.clear()
            else:
                self.auto_mode.setChecked(True)
                self.change_mode_control_enable('auto')
                self.manual_menu.clear()
        elif btn.text() == "????????????".decode('utf-8'):
            if btn.isChecked() == True:
                self.manual_mode.setChecked(False)
                self.change_mode_control_enable('auto')
                self.manual_menu.clear()
            else:
                self.manual_mode.setChecked(True)
                self.change_mode_control_enable('manual')
                self.auto_menu.clear()

    def change_all_control_enable(self, msg):
        if msg == 0:
            self.set_all_control_disable()
            self.menu_label.setPixmap(self.initial_pic)
            self.dish_reco_result_label.setPixmap(self.initial_pic_1)
        if msg == 1:
            self.auto_mode.setEnabled(True)
            self.manual_mode.setEnabled(True)
            self.generate_menu_button.setEnabled(True)
            self.generate_menu_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.save_menu_button.setEnabled(True)
            self.save_menu_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.upload_menu_button.setEnabled(True)
            self.upload_menu_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.save_menu_path_button.setEnabled(True)
            self.save_menu_path_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')



    def change_mode_control_enable(self, mode):
        if mode == 'auto':
            self.reco_number_button.setEnabled(False)
            self.car_back_copy.setEnabled(False)
            self.car_forward_copy.setEnabled(False)
            self.reco_dish_button.setEnabled(False)
            self.dish_location_input.setEnabled(False)

            self.start_scan_button.setEnabled(True)
            self.start_scan_label_button.setEnabled(True)
            self.move_frequency_input.setEnabled(True)
            self.move_interval_input.setEnabled(True)

            self.reco_number_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.car_back_copy.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.car_forward_copy.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.reco_dish_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.start_scan_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.start_scan_label_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')


        elif mode == 'manual':
            self.reco_number_button.setEnabled(True)
            self.car_back_copy.setEnabled(True)
            self.car_forward_copy.setEnabled(True)
            self.reco_dish_button.setEnabled(True)
            self.dish_location_input.setEnabled(True)

            self.start_scan_button.setEnabled(False)
            self.start_scan_label_button.setEnabled(False)
            self.move_frequency_input.setEnabled(False)
            self.move_interval_input.setEnabled(False)

            self.reco_number_button.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.car_back_copy.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.car_forward_copy.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.reco_dish_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.start_scan_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.start_scan_label_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')

class Dig_Dish_Page(QMainWindow, Ui_Dig_dish_Form):
    def __init__(self, system_monitor_thread, emergency_stop_thread, camera_thread):
        super(Dig_Dish_Page, self).__init__()
        self.setupUi(self)
        self.timer = QTimer()
        self.timer.timeout.connect(self.showtime)
        self.timer.start(1000)
        self.bridge = CvBridge()

        self.all_control_disable()
        temp_time = QDateTime.currentDateTime()
        self.timedisplay = temp_time.toString("hh:mm:ss")

        self.camera_image_copy2 = camera_thread
        self.system_status_thread = system_monitor_thread
        self.system_status_variable = False

        self.dig_dish_thread = final_ur3ur10_alone_work_qt()
        self.emergency_stop_thread_copy = emergency_stop_thread
        self.order_list_text.append(str(self.timedisplay) + ': Pointseg Model Weights Loading Finish')

        self.ur3_add_bowl_thread = ur3_add_bowl_Worker()

        self.CreateSignalSlot()

        self.initial_pic = QPixmap('./kangli.png')
        self.dig_show_label.setPixmap(self.initial_pic)
        self.dig_show_label.setScaledContents(True)

        self.current_menu_list.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.bowl_num_input.setRange(0, 10)
        self.choose_dish_list = []
        self.bowl_number = 0

    def CreateSignalSlot(self):
        self.updata_menu_button.clicked.connect(self.updata_dish_list)
        self.acquire_order_button.clicked.connect(self.acquire_order_work)
        self.clear_dig_txt.clicked.connect(self.clear_message_work)

        self.system_status_thread.system_status_change.connect(self.change_all_control_status)
        # self.system_status_thread.start()
        self.emergency_stop_thread_copy.dig_status_signnal.connect(self.button_control_dig_work)
        # self.emergency_stop_thread_copy.start()

        self.camera_image_copy2.aruco_result_signal.connect(self.show_raw_image)
        # self.camera_image_copy2.start()

        self.dig_dish_start_button.clicked.connect(lambda :self.dig_dish_start_work(manual_order=[self.bowl_number]))
        self.stop_dig_button.clicked.connect(self.dig_stop_work)
        self.resume_dig_button.clicked.connect(self.dig_resume_work)
        self.dig_dish_end_button.clicked.connect(self.dig_end_work)

        self.dig_dish_thread.work_status_signal.connect(self.dig_dish_status_feedback)
        self.dig_dish_thread.get_order_signal.connect(self.show_new_order_work)

        self.current_menu_list.clicked.connect(self.choose_dish)
        self.bowl_num_input.valueChanged.connect(self.updata_bowl_num)
        self.dig_choose_dish_button.clicked.connect(self.dig_choose_dish_work)

        self.add_bowl_button.clicked.connect(self.ur3_add_bowl)
        self.ur3_add_bowl_thread.add_finish_signal.connect(self.show_add_bowl_finish)

    def ur3_add_bowl(self):
        reply = QMessageBox.information(self, "??????", "??????????????????, ?????????????????????????????????",
                                        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.ur3_add_bowl_thread.start()
        else:
            self.order_list_text.append(str(self.timedisplay) + ': ????????????')

    def show_add_bowl_finish(self):
        self.order_list_text.append(str(self.timedisplay) + ': ?????????5??????')

    def choose_dish(self):
        self.choose_dish_list = []
        for i in self.current_menu_list.selectedIndexes():
            text = i.data()
            self.choose_dish_list.append(text)
        # self.choose_dish_list = str(self.choose_dish_list).decode('unicode-escape')
        self.order_list_text.append(str(self.timedisplay) + '?????????????????????' + str(self.choose_dish_list))

    def show_new_order_work(self, msg):
        self.order_list_text.append(str(self.timedisplay) + ": " + msg)

    def updata_bowl_num(self):
        self.bowl_number = self.bowl_num_input.value()

    def dig_choose_dish_work(self):
        if self.dig_dish_start_button.isEnabled():
            if len(self.choose_dish_list) != 0:
                reply = QMessageBox.information(self, "??????", "????????????" + str(self.choose_dish_list) + ", ??????????????????",
                                                QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

                if reply == QMessageBox.Yes:
                    manual_order = [len(self.choose_dish_list), self.choose_dish_list]
                    self.choose_dish_list = []
                    self.order_list_text.append(str(self.timedisplay) + "?????????????????????: " + str(manual_order))
                    self.dig_dish_start_work(manual_order)
                else:
                    self.order_list_text.append(str(self.timedisplay) + "?????????????????????")
                    self.choose_dish_list = []
            else:
                self.order_list_text.append(str(self.timedisplay) + "???????????????")
        else:
            QMessageBox.information(self, "??????", "??????????????????", QMessageBox.Ok)

    def button_control_dig_work(self, msg):
        if self.system_status_variable:
            if msg == 0:
                self.dig_dish_start_work(manual_order=[self.bowl_number])
            elif msg == 1:
                if self.dig_dish_start_button.isEnabled():
                    QMessageBox.information(self, "??????", "?????????????????????", QMessageBox.Ok)
                else:
                    self.dig_stop_work()
            elif msg == 2:
                if self.dig_dish_start_button.isEnabled():
                    QMessageBox.information(self, "??????", "?????????????????????", QMessageBox.Ok)
                else:
                    self.dig_resume_work()
            elif msg == 3:
                self.dig_end_work()
        else:
            QMessageBox.information(self, "??????", "????????????????????????", QMessageBox.Ok)


    def dig_dish_start_work(self, manual_order):
        if len(manual_order) == 1:
            reply = QMessageBox.information(self, "??????", "?????????????????????" + str(manual_order[0]) + "???, ????????????????????????",
                                            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.dig_dish_thread.receive_choose_dish_oeder(manual_order)
                self.dig_dish_thread.start()
            else:
                self.order_list_text.append(str(self.timedisplay) + "????????????")


        else:
            self.dig_dish_thread.receive_choose_dish_oeder(manual_order)
            self.dig_dish_thread.start()

    def dig_stop_work(self):
        self.dig_dish_thread.receive_stop_signal('stop')

    def dig_resume_work(self):
        self.dig_dish_thread.receive_stop_signal('resume')

    def dig_end_work(self):
        self.dig_dish_thread.receive_stop_signal('kill')


    def dig_dish_status_feedback(self, msg):
        if msg == 1:
            self.order_list_text.append(str(self.timedisplay) + ": ????????????")

            self.emergency_stop_thread_copy.is_dig_stop = True
            self.emergency_stop_thread_copy.is_dig_start = True

            self.stop_dig_button.setEnabled(False)
            self.stop_dig_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.resume_dig_button.setEnabled(True)
            self.resume_dig_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            playsound('/home/robot/sound/suspend.mp3', block=True)
        elif msg == 2:
            self.order_list_text.append(str(self.timedisplay) + ": ????????????")

            self.emergency_stop_thread_copy.is_dig_stop = False
            self.emergency_stop_thread_copy.is_dig_start = True

            self.stop_dig_button.setEnabled(True)
            self.stop_dig_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.resume_dig_button.setEnabled(False)
            self.resume_dig_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            playsound('/home/robot/sound/resume.mp3', block=True)
        elif msg == 3:
            self.order_list_text.append(str(self.timedisplay) + ": ????????????")

            self.emergency_stop_thread_copy.is_dig_stop = False
            self.emergency_stop_thread_copy.is_dig_start = False

            self.dig_dish_start_button.setEnabled(True)
            self.dig_dish_start_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.dig_dish_end_button.setEnabled(False)
            self.dig_dish_end_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.stop_dig_button.setEnabled(False)
            self.stop_dig_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.resume_dig_button.setEnabled(False)
            self.resume_dig_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.dig_choose_dish_button.setEnabled(True)
            self.dig_choose_dish_button.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            playsound('/home/robot/sound/end.mp3', block=True)

        elif msg == 0:
            self.order_list_text.append(str(self.timedisplay) + ": ????????????")

            self.emergency_stop_thread_copy.is_dig_stop = False
            self.emergency_stop_thread_copy.is_dig_start = True

            self.dig_dish_start_button.setEnabled(False)
            self.dig_dish_start_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.dig_dish_end_button.setEnabled(True)
            self.dig_dish_end_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.stop_dig_button.setEnabled(True)
            self.stop_dig_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.resume_dig_button.setEnabled(False)
            self.resume_dig_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.dig_choose_dish_button.setEnabled(False)
            self.dig_choose_dish_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            playsound('/home/robot/sound/start.mp3', block=True)

        elif msg == 4:
            self.order_list_text.append(str(self.timedisplay) + ": ????????????")

            self.emergency_stop_thread_copy.is_dig_stop = True
            self.emergency_stop_thread_copy.is_dig_start = True

            self.stop_dig_button.setEnabled(False)
            self.stop_dig_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.resume_dig_button.setEnabled(True)
            self.resume_dig_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            playsound('/home/robot/sound/add.mp3', block=True)

        elif msg == 5:
            playsound('/home/robot/sound/button.mp3', block=True)

    def updata_dish_list(self):
        slm = QStringListModel()
        fw = open("/home/robot/photo/memu_weizhi_test.txt", "r")
        content = fw.read()
        result = eval(content)
        memu = []
        for key in result:
            memu.append(key)
        slm.setStringList(memu)
        self.current_menu_list.setModel(slm)

    def acquire_order_work(self):
        url = "http://canteen.cailh.club/sell/robot/order?robotId=0&canteen=0"
        result = requests.get(url)
        data = json.loads(result.text)
        food = food_msg()
        if data['success']:
            food.id = data['id']
            food.food = json.dumps(data['food'])
            self.order_list_text.append(str(self.timedisplay) + ": ??????" + str(json.loads(food.food)))

        else:
            self.order_list_text.append(str(self.timedisplay) + ": ????????????")

    def show_raw_image(self, msg):
        label_width = self.dig_show_label.width()
        label_height = self.dig_show_label.height()
        temp_imgSrc = QImage(msg, msg.shape[1], msg.shape[0], msg.shape[1] * 3, QImage.Format_RGB888)
        pixmap_imgSrc = QPixmap.fromImage(temp_imgSrc).scaled(label_width, label_height)

        self.dig_show_label.setPixmap(pixmap_imgSrc)

    def showtime(self):
        time = QDateTime.currentDateTime()
        self.timedisplay = time.toString("hh:mm:ss")

    def clear_message_work(self):
        self.order_list_text.clear()

    def change_all_control_status(self, msg):
        if msg == 0:
            self.all_control_disable()
            self.dig_show_label.setPixmap(self.initial_pic)
            self.system_status_variable = False

        elif msg == 1:
            self.dig_choose_dish_button.setEnabled(True)
            self.add_bowl_button.setEnabled(True)

            self.dig_dish_start_button.setEnabled(True)

            self.stop_dig_button.setEnabled(False)

            self.dig_choose_dish_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.add_bowl_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')

            self.dig_dish_start_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')

            self.stop_dig_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.system_status_variable = True

    def all_control_disable(self):
        self.dig_choose_dish_button.setEnabled(False)
        self.add_bowl_button.setEnabled(False)
        self.dig_dish_end_button.setEnabled(False)
        self.dig_dish_start_button.setEnabled(False)
        self.resume_dig_button.setEnabled(False)
        self.stop_dig_button.setEnabled(False)


        self.dig_choose_dish_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.add_bowl_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.dig_dish_end_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.dig_dish_start_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.resume_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.stop_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')

class Main_Page(QMainWindow, Ui_All_MainWindow):

    def __init__(self, Parent=None):
        super(Main_Page, self).__init__()
        self.setupUi(self)
        # self.setWindowFlags(Qt.FramelessWindowHint)
        self.qsl = QStackedLayout(self.frame)


        self.initial_work_class = Initial_Work_Page()
        self.generate_menu_class = Generate_Menu_Page(self.initial_work_class.system_monitor_thread, self.initial_work_class.camera_image)
        self.dig_dish_class = Dig_Dish_Page(self.initial_work_class.system_monitor_thread, self.initial_work_class.emergency_stop_thread, self.initial_work_class.camera_image)

        self.qsl.addWidget(self.initial_work_class)
        self.qsl.addWidget(self.generate_menu_class)
        self.qsl.addWidget(self.dig_dish_class)

        self.CreateSignalSlot()

        self.Initial_button.setStyleSheet('background-color:rgb(200,0,0,0); color:red')
        self.Menu_button.setStyleSheet('background-color:rgb(200,0,0,0); color:gray')
        self.DIg_dish_button.setStyleSheet('background-color:rgb(200,0,0,0); color:gray')

    def CreateSignalSlot(self):
        self.Initial_button.clicked.connect(self.switch)
        self.Menu_button.clicked.connect(self.switch)
        self.DIg_dish_button.clicked.connect(self.switch)

    def switch(self):
        sender = self.sender().objectName()

        index = {
            "Initial_button": 0,
            "Menu_button": 1,
            "DIg_dish_button": 2,
        }

        self.qsl.setCurrentIndex(index[sender])
        if(index[sender] == 0):
            self.Initial_button.setStyleSheet('background-color:rgb(200,0,0,0); color:red')
            self.Menu_button.setStyleSheet('background-color:rgb(200,0,0,0); color:gray')
            self.DIg_dish_button.setStyleSheet('background-color:rgb(200,0,0,0); color:gray')
        elif (index[sender] == 1):
            self.Initial_button.setStyleSheet('background-color:rgb(200,0,0,0); color:gray')
            self.Menu_button.setStyleSheet('background-color:rgb(200,0,0,0); color:red')
            self.DIg_dish_button.setStyleSheet('background-color:rgb(200,0,0,0); color:gray')
        elif (index[sender] == 2):
            self.Initial_button.setStyleSheet('background-color:rgb(200,0,0,0); color:gray')
            self.Menu_button.setStyleSheet('background-color:rgb(200,0,0,0); color:gray')
            self.DIg_dish_button.setStyleSheet('background-color:rgb(200,0,0,0); color:red')


    def closeEvent(self, event):
        reply = QMessageBox.question(self, 'Exit', '??????????????????????', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            if self.initial_work_class.sys_end.isEnabled():
                QMessageBox.information(self, '??????', '???????????????', QMessageBox.Yes)
                event.ignore()
            else:
                event.accept()
        else:
            event.ignore()

class WrenchPlot_Worker(QThread):
    wrench_show_status_signal = pyqtSignal(int)

    def __init__(self):
        QThread.__init__(self)
        self.stop_show = False

    def run(self):
        wrench_plot_process = multiprocessing.Process(target=self.wrench_show)
        wrench_plot_process.start()

        wrench_plot_process.join()
        self.wrench_show_status_signal.emit(1)

    def wrench_show(self):
        app = QApplication(sys.argv)
        self.wrench_show_page = Wrench_plot_Page()
        app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
        self.wrench_show_page.show()
        # sys.exit(app.exec_())


class CameraRosNode(QThread):
    aruco_result_signal = pyqtSignal(np.ndarray)
    raw_image_signal = pyqtSignal(np.ndarray) #?????? signal????????? ???ros?????????????????? ???????????????????????????

    def __init__(self):
        QThread.__init__(self)  #????????????
        self.end_process = False

        # rospy.Subscriber('/cam_front/csi_cam/image_raw/compressed', CompressedImage, self.callback_compressed_image)

    def run(self):
        self.signal_aruco_send, self.signal_aruco_receive = multiprocessing.Pipe(True)
        self.signal_image_send, self.signal_image_receive = multiprocessing.Pipe(True)
        camera_data_process = multiprocessing.Process(target=self.subscriber_topic,
                                                      args=(self.signal_aruco_send, self.signal_image_send,))
        camera_data_process.start()
        self.process_pid = psutil.Process(camera_data_process.pid)
        self.callback_data()
        camera_data_process.join()
        print "camera end"


    def subscriber_topic(self, signal_aruco_send, signal_image_send):
        rospy.init_node('camera_ros_node', anonymous=True) #??????ros??????
        # rospy.Subscriber('/camera2/color/image_raw', Image, self.callback_raw_image)
        rospy.Subscriber('/camera2/color/image_raw', Image, self.callback_aruco_result, (signal_aruco_send,))
        rospy.spin()  # ????????????

    def callback_data(self):
        while not self.end_process:
            # print aruco_result.shape
            aruco_result = self.signal_aruco_receive.recv()
            self.aruco_result_signal.emit(aruco_result)

        if self.end_process:
            self.process_pid.kill()

    def callback_aruco_result(self, data, args):
        signal_aruco_send = args[0]
        bridge = CvBridge()
        self.aruco_result_opencv = bridge.imgmsg_to_cv2(data, "bgr8")
        self.aruco_result_opencv = cv2.cvtColor(self.aruco_result_opencv, cv2.COLOR_BGR2RGB)
        signal_aruco_send.send(self.aruco_result_opencv)
        # self.aruco_result_signal.emit(self.aruco_result_opencv)

    def callback_raw_image(self, data):
        bridge = CvBridge()
        self.raw_image_opencv = bridge.imgmsg_to_cv2(data, "bgr8")
        self.raw_image_opencv = cv2.cvtColor(self.raw_image_opencv, cv2.COLOR_BGR2RGB)
        # self.raw_image_signal.emit(self.raw_image_opencv)

class initial_Worker(QThread):
    # sinOut = pyqtSignal(str)  # ?????????????????????

    def __init__(self, parent=None):
        super(initial_Worker, self).__init__(parent)
        self.work = True
        self.num = 0

    def run(self):
        camera = subprocess.Popen('roslaunch realsense2_camera rs_camera_vegetable.launch', shell=True)
        rospy.sleep(2)
        ag95_gripper = subprocess.Popen('roslaunch dh_hand_driver dh_hand_controller.launch', shell=True)
        rospy.sleep(2)
        dual_arm = subprocess.Popen('roslaunch dual_arm_car_grippers_modern_driver '
                                    'dual_arm_car_grippers_fts_metal_publish_alone.launch', shell=True)
        rospy.sleep(2)


class ur3_add_bowl_Worker(QThread):
    add_finish_signal = pyqtSignal(int)
    def __init__(self, parent=None):
        super(ur3_add_bowl_Worker, self).__init__(parent)

    def run(self):
        ur3_add_bowl_action.ur3_add_bowl_work(0)
        self.add_finish_signal.emit(0)

class close_Worker(QThread):
    def __init__(self, parent=None):
        super(close_Worker, self).__init__(parent)
        self.work = True
        self.num = 0

    def run(self):
        kill = subprocess.Popen('rosnode kill -a', shell=True)
        rospy.sleep(1)

class emergency_stop_Worker(QThread):
    emeregency_stop_signal = pyqtSignal()
    diconnect_ser_signal = pyqtSignal()

    emergency_button_signal = pyqtSignal(int)
    dig_status_signnal = pyqtSignal(int)

    def __init__(self, parent=None):
        super(emergency_stop_Worker, self).__init__(parent)
        self.contol_messageBox = True
        self.is_dig_stop = False
        self.is_dig_start = False
        portx = pause_usb_name
        bps = 9600
        timex = None
        try:
            self.ser = serial.Serial(portx, bps, timeout=timex)
            self.is_ser = True
        except:
            self.is_ser = False

    def run(self):

        if self.is_ser == True:
            self.check_emergency_button()
            while 1:
                data = binascii.b2a_hex(self.ser.read())
                if data == "14":
                    self.emeregency_stop_signal.emit()
                    self.check_emergency_button()
                if data == '13':
                    if not self.is_dig_stop:
                        self.dig_status_signnal.emit(1)
                        # is_dig_stop = True
                    elif self.is_dig_stop:
                        self.dig_status_signnal.emit(2)
                        # is_dig_stop = False
                if data == "10":
                    # self.dig_status_signnal.emit(0)
                    # is_dig_stop = False
                    if not self.is_dig_start:
                        self.dig_status_signnal.emit(0)
                        # self.is_dig_start = True
                        # self.is_dig_stop = False
                    else:
                        self.dig_status_signnal.emit(3)
                        # is_dig_start = False


                # if data == "11":
                #     self.dig_status_signnal.emit(3)
        else:
            self.diconnect_ser_signal.emit()

    def check_emergency_button(self):

        while 1:
            self.ser.write('\x20')
            button_status = binascii.b2a_hex(self.ser.read())
            if button_status == "15":
                if self.contol_messageBox == False:
                    self.emergency_button_signal.emit(0)
                break
            elif button_status == "14":
                if self.contol_messageBox:
                    self.emergency_button_signal.emit(1)
                    self.contol_messageBox = False

    def messageBox_close(self):
        self.contol_messageBox = True

class monitor_power_status(QThread):
    power_change = pyqtSignal(int)
    mqtt_server_fail = pyqtSignal()
    def __init__(self, parent=None):
        super(monitor_power_status, self).__init__(parent)
        power_status_client = mqtt.Client("powewr-monitor")
        try:
            power_status_client.connect(HOST, PORT, 60)
            power_status_client.loop_start()
            self.mqtt_server_connect = True
        except:
            self.mqtt_server_connect = False

        self.power_status_feedback = '0.0000'
        self.temp_status = '0.0000'

        if self.mqtt_server_connect == True:
            self.power_on_subscribe(power_status_client)

    def power_on_subscribe(self, client):
        client.subscribe("/Power_status", 0)
        client.on_message = self.power_on_message_come

    def power_on_message_come(self, mqtt_client, userdata, msg):
        self.power_status_feedback = msg.payload.split("\r\n")[0]

    def run(self):
        if self.mqtt_server_connect == False:
            self.mqtt_server_fail.emit()
        else:
            while True:
                rospy.sleep(0.5)
                if self.power_status_feedback != self.temp_status:
                    self.temp_status = self.power_status_feedback
                    if self.power_status_feedback == "1.0000":
                        self.power_change.emit(1)
                    elif self.power_status_feedback == "0.0000":
                        self.power_change.emit(0)

class monitor_system_status(QThread):
    system_status_change = pyqtSignal(int)
    def __init__(self, parent=None):
        super(monitor_system_status, self).__init__(parent)
        self.cameraConnect = False
        self.cameraUR = False
        self.end_process = False

        self.temp_cameraConnect = False
        self.temp_cameraUR = False
        self.signal_system_status_send, self.signal_system_status_receive = multiprocessing.Pipe(True)

    # def create_rosnode(self):
    #     rospy.init_node("system_status", anonymous=False)

    def run(self):
        # self.signal_system_status_send, self.signal_system_status_receive = multiprocessing.Pipe(True)
        system_status = multiprocessing.Process(target=self.monitor_work, args=(self.signal_system_status_send, ))
        system_status.daemon = True
        system_status.start()
        self.system_monitor_process = psutil.Process(system_status.pid)
        self.receive_system_status()
        system_status.join()
        # print "systen end"

    def monitor_work(self, signal_system_status_send):
        rospy.init_node("system_status", anonymous=True)
        while True:
            # ??????
            # signal_system_status_send.send(1)
            if self.temp_cameraUR == False and self.temp_cameraConnect ==False:
                try:
                    rospy.wait_for_message("/ur3/joint_states", JointState, timeout=3)
                    print "robot success"
                    self.cameraUR = True
                except:
                    self.cameraUR = False
                try:
                    rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=3)
                    self.cameraConnect = True
                    print "camrera success"
                except:
                    self.cameraConnect = False

            elif self.temp_cameraUR == True and self.temp_cameraConnect == True:
                try:
                    rospy.wait_for_message("/ur3/joint_states", JointState, timeout=1)
                    self.cameraUR = True
                except:
                    self.cameraUR = False
                try:
                    rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=1)
                    self.cameraConnect = True
                except:
                    self.cameraConnect = False

            if self.temp_cameraConnect != self.cameraConnect and self.temp_cameraUR != self.cameraUR:
                self.temp_cameraUR = self.cameraUR
                self.temp_cameraConnect = self.cameraConnect
                if self.cameraConnect == True and self.cameraUR == True:
                    signal_system_status_send.send(1)
                if self.cameraConnect == False and self.cameraUR == False:
                    signal_system_status_send.send(0)

    def receive_system_status(self):
        while not self.end_process:
            system_status = self.signal_system_status_receive.recv()
            if system_status == 0:
                self.system_status_change.emit(0)
            elif system_status == 1:
                self.system_status_change.emit(1)
        if self.end_process:
            self.system_monitor_process.kill()

class recognize_start_loc(QThread):
    need_move_dis = pyqtSignal(float)

    def __init__(self, parent=None):
        super(recognize_start_loc, self).__init__(parent)
        self.initial_loc = -0.0340474545951 + 0.019547507 + 0.032159426

    def run(self):
        self.signal_mark_result_send, self.signal_mark_result_receive = multiprocessing.Pipe(True)
        recognize_mark_process = multiprocessing.Process(target=self.recognize_start_work, args=(self.signal_mark_result_send,))
        recognize_mark_process.start()
        self.recognize_mark_process_pid = psutil.Process(recognize_mark_process.pid)
        self.emit_reco_result()
        recognize_mark_process.join()
        print "recognize mark process end"

    def recognize_start_work(self, signal_mark_result_send):
        rospy.init_node("recognize_marker", anonymous=False)
        reference_frame = "body_link"
        ur10_group_name = "ur10_with_gripper"
        ur10_arm_spoon = moveit_commander.MoveGroupCommander(ur10_group_name, ns='ur10')

        ur10_recognize_start_joint = [-0.1650312582599085, -1.4305527845965784, 0.9730334281921387, -1.5256131331073206, -1.4697869459735315, -2.4985631147967737]

        # ?????????????????????????????????????????????
        ur10_arm_spoon.allow_replanning(True)

        # ?????????????????????????????????????????????
        ur10_arm_spoon.set_pose_reference_frame(reference_frame)

        # ????????????(????????????)?????????????????????????????????????????????
        ur10_arm_spoon.set_goal_position_tolerance(0.001)
        ur10_arm_spoon.set_goal_orientation_tolerance(0.001)

        # ???????????????????????????????????????
        ur10_arm_spoon.set_max_acceleration_scaling_factor(0.25)
        ur10_arm_spoon.set_max_velocity_scaling_factor(0.25)

        # ????????????link?????????RO
        ur10_spoon_end_effector_link = ur10_arm_spoon.get_end_effector_link()

        ur10_arm_spoon.set_joint_value_target(ur10_recognize_start_joint)
        ur10_arm_spoon.go()
        try:
            data = rospy.wait_for_message('/aruco_single/position', Vector3Stamped, timeout=1)
            print data.vector
            move_dis = (data.vector.y - self.initial_loc)
        except:
            move_dis = 0

        # self.need_move_dis.emit(move_dis)
        signal_mark_result_send.send(move_dis)

    def emit_reco_result(self):
        move_dis = self.signal_mark_result_receive.recv()
        self.need_move_dis.emit(move_dis)
        self.recognize_mark_process_pid.kill()

class auto_gen_menu(QThread):
    reco_dish_result = pyqtSignal(np.ndarray)
    food_name_signal = pyqtSignal(str)
    dish_location_odom_signal = pyqtSignal(dict)
    def __init__(self, parent=None):
        super(auto_gen_menu, self).__init__(parent)
        self.move_interval_num = 0
        self.move_frequency_num = 0
        self.signal_auto_mess_send, self.signal_auto_mess_receive = multiprocessing.Pipe(True)
        self.signal_yolo_send, self.signal_yolo_receive = multiprocessing.Pipe(True)

    def get_values(self, interval, frequency, yolo):
        self.move_interval_num = interval
        self.move_frequency_num = frequency
        self.yolo_detect = yolo
        self.signal_auto_mess_send.send([self.move_interval_num, self.move_frequency_num])
        self.signal_yolo_send.send(self.yolo_detect)

    def run(self):
        auto_menu_process = multiprocessing.Process(target=self.auto_menu_work, args=(self.signal_auto_mess_receive, self.signal_yolo_receive))
        auto_menu_process.start()
        self.auto_menu_process_pid = psutil.Process(auto_menu_process.pid)
        self.emit_auto_menu()
        auto_menu_process.join()

    def auto_menu_work(self, signal_auto_mess_receive, signal_yolo_receive):
        rospy.init_node("auto_menu_yolo", anonymous=False)
        menu_client = mqtt.Client("generate_menu_yolo")
        menu_client.connect(HOST, PORT, 60)
        menu_client.loop_start()
        bridge = CvBridge()

        move_mess = signal_auto_mess_receive.recv()
        move_one_distance = move_mess[0]
        i = move_mess[1]
        total_num = move_mess[1]

        dish_location_odom = {}
        yolo_detect = signal_yolo_receive.recv()
        while (i):
            data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=0.5)
            frame = bridge.imgmsg_to_cv2(data, "bgr8")
            image = frame[:, :, [2, 1, 0]]
            food_name, image_result = yolo_detect.detect_image(image)
            signal_auto_mess_receive.send(image_result)
            signal_auto_mess_receive.send(food_name)
            # self.reco_dish_result.emit(image_result)
            # self.food_name_signal.emit(food_name)
            dish_location_odom[food_name] = move_one_distance * (total_num - i)
            if i != 1:
                scan_dish.car_move(move_one_distance, menu_client)
            i -= 1
        rospy.sleep(0.2)
        # self.dish_location_odom_signal.emit(dish_location_odom)
        scan_dish.car_move(-(total_num-1) * move_one_distance, menu_client)
        signal_auto_mess_receive.send(dish_location_odom)
        rospy.sleep(0.5)
        menu_client.disconnect()

    def emit_auto_menu(self):
        for i in range(self.move_frequency_num):
            image_result = self.signal_auto_mess_send.recv()
            food_name = self.signal_auto_mess_send.recv()
            self.reco_dish_result.emit(image_result)
            self.food_name_signal.emit(food_name)
        menu = self.signal_auto_mess_send.recv()
        self.dish_location_odom_signal.emit(menu)
        self.auto_menu_process_pid.kill()
        print "auto mene process end"


class auto_gen_menu_with_aruco(QThread):
    reco_dish_result = pyqtSignal(np.ndarray)
    food_name_signal = pyqtSignal(str)
    dish_location_odom_signal = pyqtSignal(dict)
    def __init__(self, parent=None):
        super(auto_gen_menu_with_aruco, self).__init__(parent)
        self.move_interval_num = 0
        self.move_frequency_num = 0
        self.signal_auto_mess_send, self.signal_auto_mess_receive = multiprocessing.Pipe(True)

    def get_values(self, interval, frequency):
        self.move_interval_num = interval
        self.move_frequency_num = frequency
        self.signal_auto_mess_send.send([self.move_interval_num, self.move_frequency_num])

    def run(self):
        auto_menu_process = multiprocessing.Process(target=self.auto_menu_work, args=(self.signal_auto_mess_receive,))
        auto_menu_process.start()
        self.auto_menu_process_pid = psutil.Process(auto_menu_process.pid)
        self.emit_auto_menu()
        auto_menu_process.join()

    def auto_menu_work(self, signal_auto_mess_receive):

        rospy.init_node("auto_menu_aruco", anonymous=False)
        menu_client = mqtt.Client("generate_menu_aruco")
        menu_client.connect(HOST, PORT, 60)
        menu_client.loop_start()
        bridge = CvBridge()

        aruco_id_to_food = {587: 'LaZiJi', 583: 'JiMiHua', 584: 'ChaoPuGua', 585: 'ZhaTuDou',
                            586: 'RouPiHuangDou'}

        move_mess = signal_auto_mess_receive.recv()
        move_one_distance = move_mess[0]
        i = move_mess[1]
        total_num = move_mess[1]

        dish_location_odom = {}

        while (i):
            data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=1)
            frame = bridge.imgmsg_to_cv2(data, "bgr8")
            image = frame[:, :, [2, 1, 0]]
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            print ids
            if ids == None:
                food_name = None
            else:
                try:
                    food_name = aruco_id_to_food[int(ids[0])]
                except:
                    food_name = None
            signal_auto_mess_receive.send(image)
            signal_auto_mess_receive.send(food_name)

            dish_location_odom[food_name] = move_one_distance * (total_num - i)
            if i != 1:
                scan_dish.car_move(move_one_distance, menu_client)
            i -= 1
        rospy.sleep(0.2)
        # self.dish_location_odom_signal.emit(dish_location_odom)
        scan_dish.car_move(-(total_num-1) * move_one_distance, menu_client)
        signal_auto_mess_receive.send(dish_location_odom)
        rospy.sleep(0.5)
        menu_client.disconnect()

    def emit_auto_menu(self):
        for i in range(self.move_frequency_num):
            image_result = self.signal_auto_mess_send.recv()
            food_name = self.signal_auto_mess_send.recv()
            self.reco_dish_result.emit(image_result)
            self.food_name_signal.emit(food_name)
        menu = self.signal_auto_mess_send.recv()
        self.dish_location_odom_signal.emit(menu)
        self.auto_menu_process_pid.kill()
        print "auto mene process end"

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    myWin = Main_Page()
    myWin.show()
    sys.exit(app.exec_())