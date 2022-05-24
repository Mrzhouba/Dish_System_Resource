# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'all_window.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_All_MainWindow(object):
    def setupUi(self, All_MainWindow):
        All_MainWindow.setObjectName("All_MainWindow")
        All_MainWindow.resize(913, 627)
        self.centralwidget = QtWidgets.QWidget(All_MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame = QtWidgets.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(150, 0, 760, 600))
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.Initial_button = QtWidgets.QPushButton(self.centralwidget)
        self.Initial_button.setGeometry(QtCore.QRect(0, 0, 151, 200))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.Initial_button.setFont(font)
        self.Initial_button.setObjectName("Initial_button")
        self.Menu_button = QtWidgets.QPushButton(self.centralwidget)
        self.Menu_button.setGeometry(QtCore.QRect(0, 200, 151, 200))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.Menu_button.setFont(font)
        self.Menu_button.setObjectName("Menu_button")
        self.DIg_dish_button = QtWidgets.QPushButton(self.centralwidget)
        self.DIg_dish_button.setGeometry(QtCore.QRect(0, 400, 151, 200))
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.DIg_dish_button.setFont(font)
        self.DIg_dish_button.setObjectName("DIg_dish_button")
        All_MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(All_MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 913, 25))
        self.menubar.setObjectName("menubar")
        All_MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(All_MainWindow)
        self.statusbar.setObjectName("statusbar")
        All_MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(All_MainWindow)
        QtCore.QMetaObject.connectSlotsByName(All_MainWindow)

    def retranslateUi(self, All_MainWindow):
        _translate = QtCore.QCoreApplication.translate
        All_MainWindow.setWindowTitle(_translate("All_MainWindow", "MainWindow"))
        self.Initial_button.setText(_translate("All_MainWindow", "初始化"))
        self.Menu_button.setText(_translate("All_MainWindow", "生成菜单"))
        self.DIg_dish_button.setText(_translate("All_MainWindow", "打菜"))

