# -*- coding: utf-8 -*-
import sys, subprocess, rospy, time
import serial, binascii
import paho.mqtt.client as mqtt
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import qdarkstyle
import multiprocessing

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

from sensor_msgs.msg import CompressedImage,Image, JointState
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
import cv2
import requests, json
from robot_order.msg import food_msg

sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/assist_modules')
from serial_drop_bowl import drop_bowl
drop_bowl_instance = drop_bowl()
from recognize_start_loc import recognize_marker
recognize_marker = recognize_marker()
from scan_dish import scan_dish
scan_dish = scan_dish()
from dual_arm_reset import *
UR3_reset = UR3_reset()
UR10_reset = UR10_reset()
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish')
from yolo import YOLO
# sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pyqt5_test/final_control/dual_arm_dig_dish')
# from final_dig_qt import final_ur3ur10_alone_work_qt
# final_ur3ur10_alone_work_qt = final_ur3ur10_alone_work_qt()

from Initial_work import Ui_Initial_Form
from generate_menu import Ui_Menu_Form
# from dig_dish import Ui_Dig_dish_Form
from all_window import Ui_All_MainWindow

HOST = "192.168.1.176"
PORT = 1883
Initial_loc = -0.048675943166
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

        temp_time = QDateTime.currentDateTime()
        self.timedisplay = temp_time.toString("hh:mm:ss")
        self.CreateSignalSlot()

        pic = QPixmap('./hangdian.jpg')
        self.aruco_label.setPixmap(pic)
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
        self.system_monitor_thread.create_rosnode()

        self.camera_image = CameraRosNode()
        self.camera_image.aruco_result_signal.connect(self.show_aruco_result)
        # self.camera_image.start()

        self.sys_start.clicked.connect(self.start_initial_thread)

        self.sys_end.clicked.connect(self.start_close_thread)

        self.power_monitor_thread.power_change.connect(self.power_status_switch)
        self.power_monitor_thread.mqtt_server_fail.connect(self.mqtt_connect_warning)
        self.system_monitor_thread.system_status_change.connect(self.system_status_switch)

        self.power_switch.clicked.connect(self.car_power_control)
        self.emergency_stop_thread.emeregency_stop_signal.connect(self.emergency_stop_control)
        self.emergency_stop_thread.diconnect_ser_signal.connect(self.switch_disconnect_warning)
        self.emergency_stop_thread.emergency_button_signal.connect(self.emergency_button_status)

        self.drop_bowl.clicked.connect(self.drop_bowl_control)
        self.clear_initial_txt.clicked.connect(self.clear_initial_message)

        self.reecognize_mark_button.clicked.connect(self.start_label)
        self.car_need_moveDis = 0
        self.car_to_zero_button.clicked.connect(self.car_move_startLoc)

        #dual arm reset
        self.UR10_to_zero.clicked.connect(self.ur10_move_zero)
        self.UR3_to_zero.clicked.connect(self.ur3_move_zero)

        self.emergency_stop_thread.start()
        self.power_monitor_thread.start()
        self.system_monitor_thread.start()
        self.camera_image.start()

    def ur3_move_zero(self):
        UR3_reset.initial_ur3_group()

        UR3_reset.ur3_reset()
        self.Initial_messgae_txt.append(str(self.timedisplay) + ': ur3回到初始位置')

    def ur10_move_zero(self):
        UR10_reset.initial_ur10_group()

        UR10_reset.ur10_reset()
        self.Initial_messgae_txt.append(str(self.timedisplay) + ': ur10回到初始位置')

    def car_move_startLoc(self):
        car_move_client = mqtt.Client("car-control")
        car_move_client.connect(HOST, PORT, 60)
        car_move_client.loop_start()

        scan_dish.car_move(self.car_need_moveDis, car_move_client)
        self.car_to_zero_button.setEnabled(False)
        self.car_to_zero_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset')
        self.Initial_messgae_txt.append(str(self.timedisplay) + ": 小车完成校准")

    def showtime(self):
        time=QDateTime.currentDateTime()
        self.timedisplay=time.toString("hh:mm:ss")
        # print type(str(self.timedisplay))

    def start_label(self):
        try:
            data = rospy.wait_for_message('/aruco_single/position', Vector3Stamped, timeout=1)
            self.Initial_messgae_txt.append(str(self.timedisplay) + ': 已识别到目标')
            self.car_need_moveDis = data.vector.y - Initial_loc
        except:
            self.Initial_messgae_txt.append(str(self.timedisplay) + ': 未识别到目标')
            self.car_need_moveDis = 0
        if self.car_need_moveDis != 0:
            self.Initial_messgae_txt.append(str(self.timedisplay) + ': 小车需要移动{}m'.format(self.car_need_moveDis))
            if self.power_switch.text() == 'power_off':
                self.car_to_zero_button.setEnabled(True)
                self.car_to_zero_button.setStyleSheet('color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')


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
        self.Initial_messgae_txt.append(str(self.timedisplay) + ": 紧急停止")
        self.emergency_power_off()
        self.start_close_thread()

    def switch_disconnect_warning(self):
        QMessageBox.warning(self, "警告", "控制开关未连接", QMessageBox.Yes)
        sys.exit(0)

    def emergency_button_status(self, msg):
        if msg == 1:
            self.initial_button_disable(False)
            reply = QMessageBox.warning(self, "警告", "紧急开关已按下", QMessageBox.Yes)
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
        QMessageBox.warning(self, "警告", "mqtt服务器连接失败", QMessageBox.Yes)
        sys.exit(0)

    def drop_bowl_control(self):
        bowl_client = mqtt.Client("ubuntu-bowl-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.loop_start()
        is_drop = drop_bowl_instance.send_order_drop_bowl(bowl_client)
        if is_drop == True:
            self.Initial_messgae_txt.append(str(self.timedisplay) + ": 落碗成功")
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

    def start_initial_thread(self):
        self.initial_thread.start()
        # self.car_power_on()


    def start_close_thread(self):
        self.emergency_power_off()
        self.close_thread.start()

        self.UR3_to_zero.setEnabled(False)
        self.UR3_to_zero.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.UR10_to_zero.setEnabled(False)
        self.UR10_to_zero.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.reecognize_mark_button.setEnabled(False)
        self.reecognize_mark_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')

        # self.system_monitor_thread.quit()
        # print self.system_monitor_thread.isRunning()
        # print self.system_monitor_thread.isFinished()
        # self.dig_dish_thread.quit()
        # self.Dish_Task.setEnabled(True)
        # print self.dig_dish_thread.isFinished()
        # print self.dig_dish_thread.isRunning()

    def show_aruco_result(self, msg):
        # print msg.shape
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
            self.Initial_messgae_txt.append(str(self.timedisplay) + ": 小车断电")
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
            self.Initial_messgae_txt.append(str(self.timedisplay) + ": 小车上电")

    def system_status_switch(self, msg):
        if msg == 0:
            self.Led.setEnabled(False)

            self.sys_start.setEnabled(True)
            self.sys_start.setStyleSheet(
                "color:rgb(255,96,96);border-radius: 10px; border: 2px groove gray;border-style: outset;")
            self.sys_end.setEnabled(False)
            self.sys_end.setStyleSheet("color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;")

            self.Initial_messgae_txt.append(str(self.timedisplay) + ": 系统关闭")
        if msg == 1:
            self.Led.setEnabled(True)

            self.sys_start.setEnabled(False)
            self.sys_start.setStyleSheet(
                "color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;")
            self.sys_end.setEnabled(True)
            self.sys_end.setStyleSheet(
                "color:rgb(255,96,96);border-radius: 10px; border: 2px groove gray;border-style: outset;")

            self.Initial_messgae_txt.append(str(self.timedisplay) + ": 系统启动")
            self.UR3_to_zero.setEnabled(True)
            self.UR3_to_zero.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.UR10_to_zero.setEnabled(True)
            self.UR10_to_zero.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.reecognize_mark_button.setEnabled(True)
            self.reecognize_mark_button.setStyleSheet(
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
        self.auto_generate_menu_thread = auto_gen_menu()
        self.camera_image_copy = camera_thread

        self.CreateSignalSlot()

        pic = QPixmap('./hangdian.jpg')
        self.dish_reco_result_label.setPixmap(pic)
        self.dish_reco_result_label.setScaledContents(True)

    def CreateSignalSlot(self):
        self.system_status_thread_copy.system_status_change.connect(self.change_all_control_enable)
        # self.system_status_thread.start()

        self.camera_image_copy.aruco_result_signal.connect(self.show_raw_image)

        self.auto_mode.clicked.connect(lambda :self.choose_generae_mode(self.auto_mode))
        self.manual_mode.clicked.connect(lambda: self.choose_generae_mode(self.manual_mode))

        self.clear_menu_txt.clicked.connect(self.clear_all_message)
        self.reco_dish_button.clicked.connect(self.manual_reco_dish)
        self.start_scan_button.clicked.connect(self.auto_reco_dish)
        self.move_interval_input.textChanged.connect(self.move_interval)
        self.move_frequency_input.textChanged.connect(self.move_frequency)
        self.dish_location_input.textChanged.connect(self.manual_change_loca)

        self.auto_generate_menu_thread.food_name_signal.connect(self.deal_auto_reco_result)
        self.auto_generate_menu_thread.dish_location_odom_signal.connect(self.deal_auto_reco_result)
        self.auto_generate_menu_thread.reco_dish_result.connect(self.deal_auto_reco_result)

        self.generate_menu_button.clicked.connect(self.generate_menu_work)
        self.save_menu_button.clicked.connect(self.save_menu_work)
        self.upload_menu_button.clicked.connect(self.upload_menu_work)

    def generate_menu_work(self):

        if self.auto_menu or self.manual_menu:
            if self.auto_mode.isChecked() == True:
                self.final_menu = self.auto_menu
                self.menu_message_text.append(str(self.timedisplay) + ': 自动模式生成菜单' + str(self.final_menu))
            else:
                self.final_menu = self.manual_menu
                self.menu_message_text.append(str(self.timedisplay) + ': 手动模式生成菜单' + str(self.final_menu))

        else:
                QMessageBox.information(self, "提示", "请先扫描菜品", QMessageBox.Ok)

    def save_menu_work(self):
        print self.final_menu
        if self.final_menu:
            with open("/home/robot/photo/memu_weizhi_test.txt", "wb") as f:
                f.write(str(self.final_menu) + '\n')
            self.menu_message_text.append(str(self.timedisplay) + ": 成功保存菜单至本地")
            self.final_menu.clear()
            self.auto_menu.clear()
            self.manual_menu.clear()
        else:
                QMessageBox.information(self, "提示", "请先生成菜单", QMessageBox.Ok)

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
            self.menu_message_text.append(str(self.timedisplay) + ": 成功上传菜单至微信")

        else:
            QMessageBox.information(self, "提示", "菜单为空,上传失败", QMessageBox.Ok)

    def manual_reco_dish(self):
        if self.dish_location >= 0:
            # data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=0.5)
            # frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # frame = self.current_image_np
            image = self.current_image_np
            print 1
            food_name, image_result = self.yolo_detect.detect_image(image)
            print 2
            self.show_image(image_result)
            self.menu_message_text.append(str(self.timedisplay) + ': ' + str(food_name))
            self.manual_menu[food_name] = self.dish_location
            print 3
        else:
            QMessageBox.information(self, "提示", "请先输入菜品位置", QMessageBox.Ok)

    def auto_reco_dish(self):
        if 0 < self.move_interval_num < 0.5 and 0 < self.move_frequency_num < 6:

            self.menu_message_text.append(
                str(self.timedisplay) + '：间隔 ' + str(self.move_interval_num) + ', 次数 ' + str(self.move_frequency_num))

            self.auto_generate_menu_thread.start()
            self.auto_generate_menu_thread.get_values(self.move_interval_num, self.move_frequency_num, self.yolo_detect)

        else:
            QMessageBox.information(self, "提示", "检查次数和间距", QMessageBox.Yes)

    def deal_auto_reco_result(self, msg):
        if type(msg) == unicode:
            if str(msg) == "None":
                self.menu_message_text.append(str(self.timedisplay) + ': 未识别到菜品')
            else:
                self.menu_message_text.append(str(self.timedisplay) + ': 识别到' + str(msg))

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
        self.generate_menu_button.setEnabled(False)
        self.save_menu_button.setEnabled(False)
        self.upload_menu_button.setEnabled(False)
        self.move_frequency_input.setEnabled(False)
        self.move_interval_input.setEnabled(False)
        self.dish_location_input.setEnabled(False)

        self.car_back_copy.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.car_forward_copy.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.reco_dish_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.start_scan_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.generate_menu_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.save_menu_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.upload_menu_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')

    def choose_generae_mode(self, btn):
        if btn.text() == "手动模式".decode('utf-8'):
            if btn.isChecked() == True:
                self.auto_mode.setChecked(False)
                self.change_mode_control_enable('manual')
            else:
                self.auto_mode.setChecked(True)
                self.change_mode_control_enable('auto')

        elif btn.text() == "自动模式".decode('utf-8'):
            if btn.isChecked() == True:
                self.manual_mode.setChecked(False)
                self.change_mode_control_enable('auto')
            else:
                self.manual_mode.setChecked(True)
                self.change_mode_control_enable('manual')

    def change_all_control_enable(self, msg):
        if msg == 0:
            self.set_all_control_disable()
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

    def change_mode_control_enable(self, mode):
        if mode == 'auto':
            self.car_back_copy.setEnabled(False)
            self.car_forward_copy.setEnabled(False)
            self.reco_dish_button.setEnabled(False)
            self.dish_location_input.setEnabled(False)

            self.start_scan_button.setEnabled(True)
            self.move_frequency_input.setEnabled(True)
            self.move_interval_input.setEnabled(True)
            self.car_back_copy.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.car_forward_copy.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.reco_dish_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.start_scan_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')


        elif mode == 'manual':
            self.car_back_copy.setEnabled(True)
            self.car_forward_copy.setEnabled(True)
            self.reco_dish_button.setEnabled(True)
            self.dish_location_input.setEnabled(True)

            self.start_scan_button.setEnabled(False)
            self.move_frequency_input.setEnabled(False)
            self.move_interval_input.setEnabled(False)
            self.car_back_copy.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.car_forward_copy.setStyleSheet(
                'color:white;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.reco_dish_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.start_scan_button.setStyleSheet(
                'color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')


# class Dig_Dish_Page(QMainWindow, Ui_Dig_dish_Form):
#     def __init__(self, system_monitor_thread, emergency_stop_thread, camera_thread):
#         super(Dig_Dish_Page, self).__init__()
#         self.setupUi(self)
#         self.timer = QTimer()
#         self.timer.timeout.connect(self.showtime)
#         self.timer.start(1000)
#         self.bridge = CvBridge()
#
#         # self.all_control_disable()
#         temp_time = QDateTime.currentDateTime()
#         self.timedisplay = temp_time.toString("hh:mm:ss")
#
#         self.camera_image_copy2 = camera_thread
#         self.system_status_thread = system_monitor_thread
#
#         self.dig_dish_thread = final_ur3ur10_alone_work_qt()
#         self.emergency_stop_thread_copy = emergency_stop_thread
#
#         self.CreateSignalSlot()
#
#     def CreateSignalSlot(self):
#         self.updata_menu_button.clicked.connect(self.updata_dish_list)
#         self.acquire_order_button.clicked.connect(self.acquire_order_work)
#         self.clear_dig_txt.clicked.connect(self.clear_message_work)
#
#         self.system_status_thread.system_status_change.connect(self.change_all_control_status)
#         # self.system_status_thread.start()
#         self.emergency_stop_thread_copy.dig_status_signnal.connect(self.button_control_dig_work)
#         # self.emergency_stop_thread_copy.start()
#
#         self.camera_image_copy2.aruco_result_signal.connect(self.show_raw_image)
#         # self.camera_image_copy2.start()
#
#         self.dig_dish_start_button.clicked.connect(self.dig_dish_start_work)
#         self.stop_dig_button.clicked.connect(self.dig_stop_work)
#         self.resume_dig_button.clicked.connect(self.dig_resume_work)
#         self.dig_dish_end_button.clicked.connect(self.dig_end_work)
#
#         self.dig_dish_thread.work_status_signal.connect(self.dig_dish_status_feedback)
#         self.dig_dish_thread.get_order_signal.connect(self.show_new_order_work)
#
#     def show_new_order_work(self, msg):
#         self.order_list_text.append(str(self.timedisplay) + ": " + str(msg))
#
#     def button_control_dig_work(self, msg):
#         if msg == 0:
#             self.dig_dish_start_work()
#         elif msg == 1:
#             self.dig_stop_work()
#         elif msg == 2:
#             self.dig_resume_work()
#         elif msg == 3:
#             self.dig_end_work()
#
#
#     def dig_dish_start_work(self):
#         self.dig_dish_thread.start()
#         self.dig_dish_start_button.setEnabled(False)
#         self.dig_dish_start_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.dig_dish_end_button.setEnabled(True)
#         self.dig_dish_end_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.stop_dig_button.setEnabled(True)
#         self.stop_dig_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.resume_dig_button.setEnabled(False)
#         self.resume_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#
#
#     def dig_stop_work(self):
#         self.dig_dish_thread.receive_stop_signal('stop')
#         self.stop_dig_button.setEnabled(False)
#         self.stop_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.resume_dig_button.setEnabled(True)
#         self.resume_dig_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#
#     def dig_resume_work(self):
#         self.dig_dish_thread.receive_stop_signal('resume')
#         self.stop_dig_button.setEnabled(True)
#         self.stop_dig_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.resume_dig_button.setEnabled(False)
#         self.resume_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#
#     def dig_end_work(self):
#         self.dig_dish_thread.receive_stop_signal('kill')
#         self.dig_dish_start_button.setEnabled(True)
#         self.dig_dish_start_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.dig_dish_end_button.setEnabled(False)
#         self.dig_dish_end_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#
#
#     def dig_dish_status_feedback(self, msg):
#         if msg == 1:
#             self.order_list_text.append(str(self.timedisplay) + ": 打菜暂停")
#         elif msg == 2:
#             self.order_list_text.append(str(self.timedisplay) + ": 打菜恢复")
#         elif msg == 3:
#             self.order_list_text.append(str(self.timedisplay) + ": 打菜结束")
#         elif msg == 0:
#             self.order_list_text.append(str(self.timedisplay) + ": 打菜开始")
#
#     def updata_dish_list(self):
#         slm = QStringListModel()
#         fw = open("/home/robot/photo/memu_weizhi_test.txt", "r")
#         content = fw.read()
#         result = eval(content)
#         memu = []
#         for key in result:
#             memu.append(key)
#         slm.setStringList(memu)
#         self.current_menu_list.setModel(slm)
#
#     def acquire_order_work(self):
#         url = "http://canteen.cailh.club/sell/robot/order?robotId=0&canteen=0"
#         result = requests.get(url)
#         data = json.loads(result.text)
#         food = food_msg()
#         if data['success']:
#             food.id = data['id']
#             food.food = json.dumps(data['food'])
#             self.order_list_text.append(str(self.timedisplay) + ": 订单" + str(json.loads(food.food)))
#
#         else:
#             self.order_list_text.append(str(self.timedisplay) + ": 没有订单")
#
#     def show_raw_image(self, msg):
#         label_width = self.dig_show_label.width()
#         label_height = self.dig_show_label.height()
#         temp_imgSrc = QImage(msg, msg.shape[1], msg.shape[0], msg.shape[1] * 3, QImage.Format_RGB888)
#         pixmap_imgSrc = QPixmap.fromImage(temp_imgSrc).scaled(label_width, label_height)
#
#         self.dig_show_label.setPixmap(pixmap_imgSrc)
#
#     def showtime(self):
#         time = QDateTime.currentDateTime()
#         self.timedisplay = time.toString("hh:mm:ss")
#
#     def clear_message_work(self):
#         self.order_list_text.clear()
#
#     def change_all_control_status(self, msg):
#         if msg == 0:
#             self.all_control_disable()
#         elif msg == 1:
#             self.dig_choose_dish_button.setEnabled(True)
#             self.add_bowl_button.setEnabled(True)
#
#             self.dig_dish_start_button.setEnabled(True)
#
#             self.stop_dig_button.setEnabled(True)
#
#             self.dig_choose_dish_button.setStyleSheet(
#                 'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#             self.add_bowl_button.setStyleSheet(
#                 'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#
#             self.dig_dish_start_button.setStyleSheet(
#                 'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#             self.resume_dig_button.setStyleSheet(
#                 'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#             self.stop_dig_button.setStyleSheet(
#                 'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#
#     def all_control_disable(self):
#         self.dig_choose_dish_button.setEnabled(False)
#         self.add_bowl_button.setEnabled(False)
#         self.dig_dish_end_button.setEnabled(False)
#         self.dig_dish_start_button.setEnabled(False)
#         self.resume_dig_button.setEnabled(False)
#         self.stop_dig_button.setEnabled(False)
#
#         self.dig_choose_dish_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.add_bowl_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.dig_dish_end_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.dig_dish_start_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.resume_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
#         self.stop_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')

class Main_Page(QMainWindow, Ui_All_MainWindow):

    def __init__(self, Parent=None):
        super(Main_Page, self).__init__()
        self.setupUi(self)
        # self.setWindowFlags(Qt.FramelessWindowHint)
        self.qsl = QStackedLayout(self.frame)

        self.initial_work_class = Initial_Work_Page()
        self.generate_menu_class = Generate_Menu_Page(self.initial_work_class.system_monitor_thread, self.initial_work_class.camera_image)
        # self.dig_dish_class = Dig_Dish_Page(self.initial_work_class.system_monitor_thread, self.initial_work_class.emergency_stop_thread, self.initial_work_class.camera_image)

        self.qsl.addWidget(self.initial_work_class)
        self.qsl.addWidget(self.generate_menu_class)
        # self.qsl.addWidget(self.dig_dish_class)

        self.CreateSignalSlot()

    def CreateSignalSlot(self):
        self.Initial_button.clicked.connect(self.switch)
        self.Menu_button.clicked.connect(self.switch)
        self.DIg_dish_button.setEnabled(False)

    def switch(self):
        sender = self.sender().objectName()

        index = {
            "Initial_button": 0,
            "Menu_button": 1,
            "DIg_dish_button": 2,
        }

        self.qsl.setCurrentIndex(index[sender])

    def closeEvent(self, event):
        reply = QMessageBox.question(self, 'Exit', '你确定要退出吗?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            event.accept()  # 接受关闭事件
        else:
            event.ignore()

class CameraRosNode(QThread):
    aruco_result_signal = pyqtSignal(np.ndarray)
    raw_image_signal = pyqtSignal(np.ndarray) #创建 signal信号， 当ros接收到消息后 触发子界面数据更新

    def __init__(self):
        QThread.__init__(self)  #创建线程

        rospy.Subscriber('/camera2/color/image_raw', Image, self.callback_raw_image)
        rospy.Subscriber('/aruco_single/result', Image, self.callback_aruco_result)
        # rospy.Subscriber('/cam_front/csi_cam/image_raw/compressed', CompressedImage, self.callback_compressed_image)

    def run(self):
        rospy.spin()


    def callback_aruco_result(self, data):
        bridge = CvBridge()
        self.aruco_result_opencv = bridge.imgmsg_to_cv2(data, "bgr8")
        self.aruco_result_opencv = cv2.cvtColor(self.aruco_result_opencv, cv2.COLOR_BGR2RGB)
        self.aruco_result_signal.emit(self.aruco_result_opencv)

    def callback_raw_image(self, data):
        bridge = CvBridge()
        self.raw_image_opencv = bridge.imgmsg_to_cv2(data, "bgr8")
        self.raw_image_opencv = cv2.cvtColor(self.raw_image_opencv, cv2.COLOR_BGR2RGB)
        self.raw_image_signal.emit(self.raw_image_opencv)


class initial_Worker(QThread):
    # sinOut = pyqtSignal(str)  # 自定义一个信号

    def __init__(self, parent=None):
        super(initial_Worker, self).__init__(parent)
        self.work = True
        self.num = 0

    def run(self):
        dual_arm = subprocess.Popen('roslaunch dual_arm_car_grippers_modern_driver dual_arm_car_grippers_metal_publish_alone.launch', shell=True)
        rospy.sleep(1)
        camera = subprocess.Popen('roslaunch realsense2_camera rs_camera_vegetable.launch', shell=True)
        rospy.sleep(1)
        ag95_gripper = subprocess.Popen('roslaunch dh_hand_driver dh_hand_controller.launch', shell=True)
        rospy.sleep(1)

class close_Worker(QThread):

    def __init__(self, parent=None):
        super(close_Worker, self).__init__(parent)
        self.work = True
        self.num = 0

    def run(self):
        acquire = subprocess.Popen('rosnode list', shell=True, stdout=subprocess.PIPE)
        result = (acquire.stdout.read().split('\n'))
        for i in range(len(result) - 1):
            if result[i][1] != 's':
                kill = subprocess.Popen('rosnode kill ' + result[i], shell=True)
                rospy.sleep(0.1)

        rospy.sleep(1)

class emergency_stop_Worker(QThread):
    emeregency_stop_signal = pyqtSignal()
    diconnect_ser_signal = pyqtSignal()

    emergency_button_signal = pyqtSignal(int)
    dig_status_signnal = pyqtSignal(int)

    def __init__(self, parent=None):
        super(emergency_stop_Worker, self).__init__(parent)
        self.contol_messageBox = True
        portx = "/dev/ttyUSB1"
        bps = 9600
        timex = None
        try:
            self.ser = serial.Serial(portx, bps, timeout=timex)
            self.is_ser = True
        except:
            self.is_ser = False

    def run(self):
        is_dig_stop = False
        is_dig_start = False

        if self.is_ser == True:
            self.check_emergency_button()
            while 1:
                data = binascii.b2a_hex(self.ser.read())
                if data == "14":
                    self.emeregency_stop_signal.emit()
                    self.check_emergency_button()
                if data == '13':
                    if not is_dig_stop:
                        self.dig_status_signnal.emit(1)
                        is_dig_stop = True
                    elif is_dig_stop:
                        self.dig_status_signnal.emit(2)
                        is_dig_stop = False

                if data == "12":
                    if not is_dig_start:
                        self.dig_status_signnal.emit(0)
                        is_dig_start = True
                        is_dig_stop = False
                    elif is_dig_start:
                        self.dig_status_signnal.emit(3)
                        is_dig_start = False

                # if data == "11":
                #     self.dig_status_signnal.emit(0)
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

        self.temp_cameraConnect = False
        self.temp_cameraUR = False
    def create_rosnode(self):
        rospy.init_node("system_status", anonymous=False)

    def run(self):

        while True:
            if self.temp_cameraUR == False and self.temp_cameraConnect ==False:
                try:
                    rospy.wait_for_message("/ur3/joint_states", JointState, timeout=3)
                    self.cameraUR = True
                except:
                    self.cameraUR = False
                try:
                    rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=3)
                    self.cameraConnect = True
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
                    self.system_status_change.emit(1)
                if self.cameraConnect == False and self.cameraUR == False:
                    self.system_status_change.emit(0)




class recognize_start_loc(QThread):
    need_move_dis = pyqtSignal(float)
    def __init__(self, parent=None):
        super(recognize_start_loc, self).__init__(parent)
        self.initial_loc = -0.048675943166
    def run(self):
        try:
            data = rospy.wait_for_message('/aruco_single/position', Vector3Stamped, timeout=5)
            print data.vector
            move_dis = data.vector.y - self.initial_loc
        except:
            move_dis = 0
        if move_dis != 0:
            self.need_move_dis.emit(move_dis)

class auto_gen_menu(QThread):
    reco_dish_result = pyqtSignal(np.ndarray)
    food_name_signal = pyqtSignal(str)
    dish_location_odom_signal = pyqtSignal(dict)
    def __init__(self, parent=None):
        super(auto_gen_menu, self).__init__(parent)
        self.move_interval_num = 0
        self.move_frequency_num = 0


    def get_values(self, interval, frequency, yolo):
        self.move_interval_num = interval
        self.move_frequency_num = frequency
        self.yolo_detect = yolo

    def run(self):
        menu_client = mqtt.Client("generate_menu")
        menu_client.connect(HOST, PORT, 60)
        menu_client.loop_start()
        bridge = CvBridge()


        move_one_distance = self.move_interval_num
        i = self.move_frequency_num
        total_num = self.move_frequency_num

        dish_location_odom = {}

        while (i):
            print 1
            data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=0.5)
            frame = bridge.imgmsg_to_cv2(data, "bgr8")
            image = frame[:, :, [2, 1, 0]]
            print 2
            food_name, image_result = self.yolo_detect.detect_image(image)
            print 3
            # signal_auto_mess_receive.send(image_result)
            # signal_auto_mess_receive.send(food_name)
            self.reco_dish_result.emit(image_result)
            self.food_name_signal.emit(food_name)
            print 4
            dish_location_odom[food_name] = [move_one_distance * (total_num - i)]
            if i != 1:
                scan_dish.car_move(move_one_distance, menu_client)
            i -= 1
        rospy.sleep(0.2)
        # signal_auto_mess_receive.send(dish_location_odom)
        self.dish_location_odom_signal.emit(dish_location_odom)
        scan_dish.car_move(-(total_num-1) * move_one_distance, menu_client)
        rospy.sleep(0.5)
        menu_client.disconnect()



if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    myWin = Main_Page()
    myWin.show()
    sys.exit(app.exec_())