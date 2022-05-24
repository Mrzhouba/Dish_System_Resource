# -*- coding: utf-8 -*-
import sys, subprocess, rospy, time
import serial, binascii
import paho.mqtt.client as mqtt
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import qdarkstyle
import multiprocessing, psutil

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
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish')
from yolo import YOLO

sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/pyqt5_test/final_control/dual_arm_dig_dish')
from final_dig_qt import final_ur3ur10_alone_work_qt
# final_ur3ur10_alone_work_qt = final_ur3ur10_alone_work_qt()


from dig_dish import Ui_Dig_dish_Form
from all_window import Ui_All_MainWindow

HOST = "192.168.1.176"
PORT = 1883
Initial_loc = -0.048675943166


class Dig_Dish_Page(QMainWindow, Ui_Dig_dish_Form):
    def __init__(self):
        super(Dig_Dish_Page, self).__init__()
        self.setupUi(self)
        self.timer = QTimer()
        self.timer.timeout.connect(self.showtime)
        self.timer.start(1000)
        self.bridge = CvBridge()

        self.all_control_disable()
        temp_time = QDateTime.currentDateTime()
        self.timedisplay = temp_time.toString("hh:mm:ss")

        self.yolo_detect = YOLO()

        self.dig_dish_thread = final_ur3ur10_alone_work_qt()
        self.power_monitor_thread = monitor_power_status()
        self.camera_data = CameraRosNode()

        self.CreateSignalSlot()

    def CreateSignalSlot(self):
        self.updata_menu_button.clicked.connect(self.updata_dish_list)
        self.acquire_order_button.clicked.connect(self.acquire_order_work)
        self.clear_dig_txt.clicked.connect(self.clear_message_work)

        self.power_monitor_thread.power_change.connect(self.change_all_control_status)
        self.camera_data.aruco_result_signal.connect(self.show_raw_image)

        self.dig_dish_start_button.clicked.connect(self.dig_dish_start_work)
        self.stop_dig_button.clicked.connect(self.dig_stop_work)
        self.resume_dig_button.clicked.connect(self.dig_resume_work)
        self.dig_dish_end_button.clicked.connect(self.dig_end_work)

        self.dig_dish_thread.work_status_signal.connect(self.dig_dish_status_feedback)
        self.dig_dish_thread.get_order_signal.connect(self.show_new_order_work)

        self.power_monitor_thread.start()
        self.camera_data.start()

    def manual_reco_dish(self):

        # data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=0.5)
        # frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # frame = self.current_image_np
        image = self.current_image_np
        print 1
        food_name, image_result = self.yolo_detect.detect_image(image)
        print 2
        print food_name
        self.order_list_text.append(str(self.timedisplay) + ': ' + str(food_name))

        print 3


    def show_new_order_work(self, msg):
        self.order_list_text.append(str(self.timedisplay) + ": " + str(msg))

    def button_control_dig_work(self, msg):
        if msg == 0:
            self.dig_dish_start_work()
        elif msg == 1:
            self.dig_stop_work()
        elif msg == 2:
            self.dig_resume_work()
        elif msg == 3:
            self.dig_end_work()


    def dig_dish_start_work(self):
        self.dig_dish_thread.start()
        self.dig_dish_start_button.setEnabled(False)
        self.dig_dish_start_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.dig_dish_end_button.setEnabled(True)
        self.dig_dish_end_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.stop_dig_button.setEnabled(True)
        self.stop_dig_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.resume_dig_button.setEnabled(False)
        self.resume_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')


    def dig_stop_work(self):
        self.dig_dish_thread.receive_stop_signal('stop')
        self.stop_dig_button.setEnabled(False)
        self.stop_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.resume_dig_button.setEnabled(True)
        self.resume_dig_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')

    def dig_resume_work(self):
        self.dig_dish_thread.receive_stop_signal('resume')
        self.stop_dig_button.setEnabled(True)
        self.stop_dig_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.resume_dig_button.setEnabled(False)
        self.resume_dig_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')

    def dig_end_work(self):
        self.dig_dish_thread.receive_stop_signal('kill')
        self.dig_dish_start_button.setEnabled(True)
        self.dig_dish_start_button.setStyleSheet('color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
        self.dig_dish_end_button.setEnabled(False)
        self.dig_dish_end_button.setStyleSheet('color:gray;border-radius: 10px; border: 2px groove gray;border-style: outset;')


    def dig_dish_status_feedback(self, msg):
        if msg == 1:
            self.order_list_text.append(str(self.timedisplay) + ": 打菜暂停")
        elif msg == 2:
            self.order_list_text.append(str(self.timedisplay) + ": 打菜恢复")
        elif msg == 3:
            self.order_list_text.append(str(self.timedisplay) + ": 打菜结束")
        elif msg == 0:
            self.order_list_text.append(str(self.timedisplay) + ": 打菜开始")

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
        self.camera_data.end_process = True

    def acquire_order_work(self):
        url = "http://canteen.cailh.club/sell/robot/order?robotId=0&canteen=0"
        result = requests.get(url)
        data = json.loads(result.text)
        food = food_msg()
        if data['success']:
            food.id = data['id']
            food.food = json.dumps(data['food'])
            self.order_list_text.append(str(self.timedisplay) + ": 订单" + str(json.loads(food.food)))

        else:
            self.order_list_text.append(str(self.timedisplay) + ": 没有订单")
        self.manual_reco_dish()

    def show_raw_image(self, msg):
        self.current_image_np = msg
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
        elif msg == 1:
            self.dig_choose_dish_button.setEnabled(True)
            self.add_bowl_button.setEnabled(True)

            self.dig_dish_start_button.setEnabled(True)

            self.stop_dig_button.setEnabled(True)

            self.dig_choose_dish_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.add_bowl_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')

            self.dig_dish_start_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.resume_dig_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')
            self.stop_dig_button.setStyleSheet(
                'color:lightblue;border-radius: 10px; border: 2px groove gray;border-style: outset;')

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


        self.dig_dish_class = Dig_Dish_Page()

        self.qsl.addWidget(self.dig_dish_class)

        self.CreateSignalSlot()

    def CreateSignalSlot(self):
        self.Initial_button.setEnabled(False)
        self.Menu_button.setEnabled(False)
        self.DIg_dish_button.clicked.connect(self.switch)

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
        self.end_process = False
        # self.yolo_detect = YOLO()
        # rospy.Subscriber('/cam_front/csi_cam/image_raw/compressed', CompressedImage, self.callback_compressed_image)

    def run(self):
        self.signal_aruco_send, self.signal_aruco_receive = multiprocessing.Pipe(True)
        self.signal_image_send, self.signal_image_receive = multiprocessing.Pipe(True)
        camera_data_process = multiprocessing.Process(target=self.subscriber_topic,
                                                      args=(self.signal_aruco_send, self.signal_image_send,))
        camera_data_process.start()
        self.process_pid =  psutil.Process(camera_data_process.pid)
        self.callback_data()
        camera_data_process.join()


    def subscriber_topic(self, signal_aruco_send, signal_image_send):
        rospy.init_node('camera_ros_node', anonymous=True) #创建ros节点
        # rospy.Subscriber('/camera2/color/image_raw', Image, self.callback_raw_image)
        rospy.Subscriber('/aruco_single/result', Image, self.callback_aruco_result)
        rospy.spin()  # 订阅数据

    def callback_data(self):
        while not self.end_process:
            aruco_result = self.signal_aruco_receive.recv()
            self.aruco_result_signal.emit(aruco_result)

        if self.end_process:
            print 11
            self.process_pid.kill()

    def callback_aruco_result(self, data):
        bridge = CvBridge()
        self.aruco_result_opencv = bridge.imgmsg_to_cv2(data, "bgr8")
        self.aruco_result_opencv = cv2.cvtColor(self.aruco_result_opencv, cv2.COLOR_BGR2RGB)
        self.signal_aruco_send.send(self.aruco_result_opencv)
        # self.aruco_result_signal.emit(self.aruco_result_opencv)

    def callback_raw_image(self, data):
        bridge = CvBridge()
        self.raw_image_opencv = bridge.imgmsg_to_cv2(data, "bgr8")
        self.raw_image_opencv = cv2.cvtColor(self.raw_image_opencv, cv2.COLOR_BGR2RGB)
        # self.raw_image_signal.emit(self.raw_image_opencv)

class monitor_power_status(QThread):
    power_change = pyqtSignal(int)
    mqtt_server_fail = pyqtSignal()
    def __init__(self, parent=None):
        super(monitor_power_status, self).__init__(parent)
        power_status_client = mqtt.Client("powewr-monitor-copy")
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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    myWin = Main_Page()
    myWin.show()
    sys.exit(app.exec_())