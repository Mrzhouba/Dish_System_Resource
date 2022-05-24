# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.Qt import *
import sys, time
import paho.mqtt.client as mqtt


class Convey_Forward_button(QtWidgets.QPushButton):
    def __init__(self, parent=None):
        super(Convey_Forward_button, self).__init__(parent)
        f = QFont("ZYSong18030", 10)
        self.setFont(f)

    def mousePressEvent(self, event):
        HOST = "192.168.20.176"
        PORT = 1883
        bowl_client = mqtt.Client("ubuntu-convey-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.publish("/convey_jog", "convey_jog,1")
        time.sleep(0.1)

    def mouseReleaseEvent(self, event):
        HOST = "192.168.20.176"
        PORT = 1883
        bowl_client = mqtt.Client("ubuntu-convey-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.publish("/convey_jog", "convey_jog,0")
        time.sleep(0.1)


class Convey_Backward_button(QtWidgets.QPushButton):
    def __init__(self, parent=None):
        super(Convey_Backward_button, self).__init__(parent)
        f = QFont("ZYSong18030", 10)
        self.setFont(f)

    def mousePressEvent(self, event):
        HOST = "192.168.20.176"
        PORT = 1883
        bowl_client = mqtt.Client("ubuntu-convey-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.publish("/convey_jog", "convey_jog,2")
        time.sleep(0.1)

    def mouseReleaseEvent(self, event):
        HOST = "192.168.20.176"
        PORT = 1883
        bowl_client = mqtt.Client("ubuntu-convey-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.publish("/convey_jog", "convey_jog,0")
        time.sleep(0.1)


class Car_Forward_button(QtWidgets.QPushButton):
    def __init__(self, parent=None):
        super(Car_Forward_button, self).__init__(parent)
        f = QFont("ZYSong18030", 10)
        self.setFont(f)

    def mousePressEvent(self, event):
        HOST = "192.168.20.176"
        PORT = 1883
        bowl_client = mqtt.Client("ubuntu-car-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.publish("/car_jog", "car_jog,1")
        time.sleep(0.1)

    def mouseReleaseEvent(self, event):
        HOST = "192.168.20.176"
        PORT = 1883
        bowl_client = mqtt.Client("ubuntu-car-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.publish("/car_jog", "car_jog,0")
        time.sleep(0.1)


class Car_Backward_button(QtWidgets.QPushButton):
    def __init__(self, parent=None):
        super(Car_Backward_button, self).__init__(parent)
        f = QFont("ZYSong18030", 10)
        self.setFont(f)

    def mousePressEvent(self, event):
        HOST = "192.168.20.176"
        PORT = 1883
        bowl_client = mqtt.Client("ubuntu-car-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.publish("/car_jog", "car_jog,2")
        time.sleep(0.1)

    def mouseReleaseEvent(self, event):
        HOST = "192.168.20.176"
        PORT = 1883
        bowl_client = mqtt.Client("ubuntu-car-control")
        bowl_client.connect(HOST, PORT, 60)
        bowl_client.publish("/car_jog", "car_jog,0")
        time.sleep(0.1)



allAttributes =   [  'colorOnBegin', 'colorOnEnd', 'colorOffBegin', 'colorOffEnd', 'colorBorderIn', 'colorBorderOut',
                     'radiusBorderOut', 'radiusBorderIn', 'radiusCircle']
allDefaultVal =   [ QColor(0, 255, 0), QColor(0, 200, 0), QColor(0, 100, 0), QColor(0, 28, 0), QColor(140, 140, 140), QColor(100, 100, 100),
                    500, 450, 400]

class MyLabel(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super(MyLabel, self).__init__(parent)
    def mouseDoubleClickEvent(self, event):
        self.w = self.width()
        if self.w == 160:
            self.setGeometry(QtCore.QRect(20, 50, 400, 300))
        else:
            self.setGeometry(QtCore.QRect(260, 50, 160, 120))

class MyLed(QAbstractButton):
    def __init__(self, parent=None):
        super(MyLed, self).__init__(parent)
        self.initUI()

    def initUI(self):
        self.setEnabled(False)
        self.scaledSize = 1000.0    #为方便计算，将窗口短边值映射为1000
        self.setLedDefaultOption()

    def setLedDefaultOption(self):
        for attr, val in zip(allAttributes, allDefaultVal):
            setattr(self, attr, val)
        self.update()

    def setLedOption(self, opt='colorOnBegin', val=QColor(0,240,0)):
        if hasattr(self, opt):
            setattr(self, opt, val)
            self.update()

    def resizeEvent(self, evt):
        self.update()

    def paintEvent(self, evt):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(Qt.black, 1))

        realSize = min(self.width(), self.height())                         #窗口的短边
        painter.translate(self.width()/2.0, self.height()/2.0)              #原点平移到窗口中心
        painter.scale(realSize/self.scaledSize, realSize/self.scaledSize)   #缩放，窗口的短边值映射为self.scaledSize
        gradient = QRadialGradient(QPointF(0, 0), self.scaledSize/2.0, QPointF(0, 0))   #辐射渐变

        #画边框外圈和内圈
        for color, radius in [(self.colorBorderOut, self.radiusBorderOut),  #边框外圈
                               (self.colorBorderIn, self.radiusBorderIn)]:   #边框内圈
            gradient.setColorAt(1, color)
            painter.setBrush(QBrush(gradient))
            painter.drawEllipse(QPointF(0, 0), radius, radius)

        # 画内圆
        gradient.setColorAt(0, self.colorOnBegin if self.isEnabled() else self.colorOffBegin)
        gradient.setColorAt(1, self.colorOnEnd if self.isEnabled() else self.colorOffEnd)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), self.radiusCircle, self.radiusCircle)


