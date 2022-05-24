#coding=UTF-8
import sys, subprocess, rospy, time
import serial, binascii
import ctypes
import paho.mqtt.client as mqtt
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from final_control import *

# -*- coding: utf-8 -*-

from PyQt5.QtCore import QThread, pyqtSignal,pyqtSlot
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

from sensor_msgs.msg import CompressedImage,Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *
import cv2

class CameraRosNode(QThread):

    raw_image_signal = pyqtSignal(np.ndarray) #创建 signal信号， 当ros接收到消息后 触发子界面数据更新
    compressed_image_signal = pyqtSignal()

    def __init__(self):
        QThread.__init__(self)  #创建线程
        rospy.init_node('camera_ros_node') #创建ros节点

        # rospy.Subscriber('/cam_front/csi_cam/image_raw/compressed', CompressedImage, self.callback_compressed_image)
        rospy.Subscriber('/aruco_single/result', Image, self.callback_raw_image)

    @pyqtSlot()
    def callback_raw_image(self, data):
        bridge = CvBridge()
        self.raw_image_opencv = bridge.imgmsg_to_cv2(data, "bgr8")
        self.raw_image_opencv = cv2.cvtColor(self.raw_image_opencv, cv2.COLOR_BGR2RGB)
        # print type(self.raw_image_opencv)
        self.raw_image_signal.emit(self.raw_image_opencv)

    def callback_compressed_image(self, data):
        self.compressed_image_array = np.fromstring(data.data, np.uint8)
        self.compressed_image_signal.emit()

    def run(self):
        rospy.spin()    #订阅数据


class show_Windown(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(show_Windown, self).__init__(parent)
        self.setupUi(self)

        self.image = CameraRosNode()

        self.CreateSignalSlot()

    def CreateSignalSlot(self):
        self.image.raw_image_signal.connect(self.show_image)
        self.pushButton.clicked.connect(self.button_evet)


    def button_evet(self):
        # QMessageBox.information(self, "提示", "open serial")
        self.textBrowser.append('hello')

    def show_image(self, msg):
        print msg.shape
        label_width = self.label.width()
        label_height = self.label.height()
        temp_imgSrc = QImage(msg, msg.shape[1], msg.shape[0], msg.shape[1] * 3, QImage.Format_RGB888)
        pixmap_imgSrc = QPixmap.fromImage(temp_imgSrc).scaled(label_width, label_height)

        self.label.setPixmap(pixmap_imgSrc)

    def closeEvent(self, event):  # 关闭窗口触发以下事件
        reply = QMessageBox.question(self, 'Exit', '你确定要退出吗?', QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            event.accept()  # 接受关闭事件
        else:
            event.ignore()



if __name__ == '__main__':
    app = QApplication(sys.argv)
    myWin = show_Windown()
    myWin.show()
    sys.exit(app.exec_())
