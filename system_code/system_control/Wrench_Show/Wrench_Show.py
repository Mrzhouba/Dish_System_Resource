# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'Wrench_Show'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Wrench(object):
    def setupUi(self, Wrench):
        Wrench.setObjectName("Wrench")
        Wrench.resize(1054, 985)
        self.centralwidget = QtWidgets.QWidget(Wrench)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton_stop = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_stop.setGeometry(QtCore.QRect(750, 890, 121, 40))
        self.pushButton_stop.setObjectName("pushButton_stop")
        self.Force = GraphicsLayoutWidget(self.centralwidget)
        self.Force.setGeometry(QtCore.QRect(20, 20, 1021, 421))
        self.Force.setObjectName("Force")
        self.Torque = GraphicsLayoutWidget(self.centralwidget)
        self.Torque.setGeometry(QtCore.QRect(20, 450, 1021, 421))
        self.Torque.setObjectName("Torque")
        self.btn_Save = QtWidgets.QPushButton(self.centralwidget)
        self.btn_Save.setGeometry(QtCore.QRect(908, 890, 121, 40))
        self.btn_Save.setObjectName("btn_Save")
        Wrench.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(Wrench)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1054, 31))
        self.menubar.setObjectName("menubar")
        Wrench.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(Wrench)
        self.statusbar.setObjectName("statusbar")
        Wrench.setStatusBar(self.statusbar)

        self.retranslateUi(Wrench)
        QtCore.QMetaObject.connectSlotsByName(Wrench)

    def retranslateUi(self, Wrench):
        _translate = QtCore.QCoreApplication.translate
        Wrench.setWindowTitle(_translate("Wrench", "MainWindow"))
        self.pushButton_stop.setText(_translate("Wrench", "Stop"))
        self.btn_Save.setText(_translate("Wrench", "Save"))

from pyqtgraph import GraphicsLayoutWidget
