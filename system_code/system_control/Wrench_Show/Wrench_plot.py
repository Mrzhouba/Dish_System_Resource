#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, subprocess, time, tf
import serial, binascii
import paho.mqtt.client as mqtt
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import qdarkstyle
import pyqtgraph.exporters
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

from sensor_msgs.msg import CompressedImage ,Image, JointState
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from PyQt5.QtGui import *

import pyqtgraph as pq


from Wrench_Show import Ui_Wrench
from child_win import Ui_MainWindow

class child(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(child, self).__init__()
        self.setupUi(self)


class Wrench_plot_Page(QMainWindow, Ui_Wrench):
    def __init__(self):
        super(Wrench_plot_Page, self).__init__()
        self.setupUi(self)
        rospy.init_node("plotCurve", anonymous=True)
        self.setGraph()
        self.addSlot()
        #
        self.curveForceX = self.forcePlot.plot([0], pen=pq.mkPen('r'), name='ForceX')
        self.curveForceY = self.forcePlot.plot([0], pen=pq.mkPen('g'), name='ForceY')
        self.curveForceZ = self.forcePlot.plot([0], pen=pq.mkPen('w'), name='ForceZ')
        self.curveTorqueX = self.torquePlot.plot([0], pen=pq.mkPen('r'), name='TorqueX')
        self.curveTorqueY = self.torquePlot.plot([0], pen=pq.mkPen('g'), name='TorqueY')
        self.curveTorqueZ = self.torquePlot.plot([0], pen=pq.mkPen('w'), name='TorqueZ')
        #

        self.timer = pq.QtCore.QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(10)
        # pq.setConfigOption()
        self.ForceXtemp = 0


        self.forceX_list = []
        self.forceY_list = []
        self.forceZ_list = []
        self.torqueX_list = []
        self.torqueY_list = []
        self.torqueZ_list = []
        rospy.Subscriber('/revise_wrench', WrenchStamped, self.callback)

    def addSlot(self):
        self.pushButton_stop.clicked.connect(self.stop_show)
        self.btn_Save.clicked.connect(self.saveCurve)

    def setGraph(self):
        self.forcePlot = self.Force.addPlot(title='Force')
        self.forcePlot.showGrid(x=False, y=True, alpha=0.5)
        self.forcePlot.addLegend()
        self.forcePlot.setYRange(-110, 0)

        self.vLineF = pq.InfiniteLine(angle=90, movable=False, )
        self.hLineF = pq.InfiniteLine(angle=0, movable=False, )
        self.forcePlot.addItem(self.vLineF, ignoreBounds=True)
        self.forcePlot.addItem(self.hLineF, ignoreBounds=True)
        self.labelF = pq.TextItem()
        self.forcePlot.addItem(self.labelF)
        self.forcePlot.scene().sigMouseMoved.connect(self.showForceData)


        self.torquePlot = self.Torque.addPlot(title='Torque')
        self.torquePlot.showGrid(x=False, y=True, alpha=0.5)
        self.torquePlot.addLegend()
        self.torquePlot.setYRange(-10, 10)

        self.vLineT = pq.InfiniteLine(angle=90, movable=False, )
        self.hLineT = pq.InfiniteLine(angle=0, movable=False, )
        self.torquePlot.addItem(self.vLineT, ignoreBounds=True)
        self.torquePlot.addItem(self.hLineT, ignoreBounds=True)
        self.labelT = pq.TextItem()
        self.torquePlot.addItem(self.labelT)
        self.torquePlot.scene().sigMouseMoved.connect(self.showTorqueData)

    def showForceData(self, evt):
        pos = evt
        if self.forcePlot.sceneBoundingRect().contains(pos):
            mousePoint = self.forcePlot.vb.mapSceneToView(pos)
            index = int(mousePoint.x())
            if index > 0 and index < len(self.forceX_list):
                # if index > 0 and index < self.MFmax:
                self.labelF.setHtml(
                    "<span style='font-size: 12pt; color: yellow'>Force={:0.01f}".format(
                        mousePoint.y()))
                self.labelF.setPos(mousePoint.x(), mousePoint.y())
            self.vLineF.setPos(mousePoint.x())
            self.hLineF.setPos(mousePoint.y())

    def showTorqueData(self, evt):
        pos = evt
        if self.torquePlot.sceneBoundingRect().contains(pos):
            mousePoint = self.torquePlot.vb.mapSceneToView(pos)
            index = int(mousePoint.x())
            if index > 0 and index < len(self.forceX_list):
                # if index > 0 and index < self.MFmax:
                self.labelT.setHtml(
                    "<span style='font-size: 12pt; color: yellow'>Torque={:0.01f}".format(
                        mousePoint.y()))
                self.labelT.setPos(mousePoint.x(), mousePoint.y())
            self.vLineT.setPos(mousePoint.x())
            self.hLineT.setPos(mousePoint.y())

    def callback(self, data):
        # a = 0.1
        #
        # self.ForceXtemp = a * data.wrench.force.x + (1 - a) * self.ForceXtemp
        # # print self.ForceXtemp
        self.forceX_list.append(data.wrench.force.x)
        self.forceY_list.append(data.wrench.force.y)
        self.forceZ_list.append(data.wrench.force.z)
        self.torqueX_list.append(data.wrench.torque.x)
        self.torqueY_list.append(data.wrench.torque.y)
        self.torqueZ_list.append(data.wrench.torque.z)

    def update_data(self):
        if len(self.torqueX_list) > 10000:
            del self.torqueX_list[0]
        if len(self.torqueY_list) > 10000:
            del self.torqueY_list[0]
        if len(self.torqueZ_list) > 10000:
            del self.torqueZ_list[0]
        if len(self.forceX_list) > 10000:
            del self.forceX_list[0]
        if len(self.forceY_list) > 10000:
            del self.forceY_list[0]
        if len(self.forceZ_list) > 10000:
            del self.forceZ_list[0]
        self.curveForceX.setData(self.forceX_list)
        self.curveForceY.setData(self.forceY_list)
        self.curveForceZ.setData(self.forceZ_list)

        self.curveTorqueX.setData(self.torqueX_list)
        self.curveTorqueY.setData(self.torqueY_list)
        self.curveTorqueZ.setData(self.torqueZ_list)

        self.torquePlot.setXRange(len(self.forceX_list) - 1000, len(self.forceX_list))
        self.torquePlot.setXRange(len(self.forceY_list) - 1000, len(self.forceY_list))
        self.torquePlot.setXRange(len(self.forceZ_list) - 1000, len(self.forceZ_list))
        self.forcePlot.setXRange(len(self.torqueX_list) - 1000, len(self.torqueX_list))
        self.forcePlot.setXRange(len(self.torqueY_list) - 1000, len(self.torqueY_list))
        self.forcePlot.setXRange(len(self.torqueZ_list) - 1000, len(self.torqueZ_list))

    def stop_show(self):
        if self.pushButton_stop.text() == 'Stop':
            self.timer.stop()
            self.pushButton_stop.setText('Start')
        else:
            self.timer.start()
            self.pushButton_stop.setText('Stop')

    def saveCurve(self):
        ForcePhoto = pq.exporters.ImageExporter(self.Force.scene())
        ForcePhoto.export(fileName="Force.png")
        TorquePhoto = pq.exporters.ImageExporter(self.Torque.scene())
        TorquePhoto.export(fileName="Torque.png")
        QMessageBox.information(self, '提示', '曲线已保存', QMessageBox.Yes)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Wrench_plot_Page()
    app.setStyleSheet(qdarkstyle.load_stylesheet_pyqt5())
    window.show()
    sys.exit(app.exec_())
