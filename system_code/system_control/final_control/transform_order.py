#coding=UTF-8


# import sys
# from PyQt5.QtWidgets import QApplication,QWidget,QVBoxLayout,QListView,QAbstractItemView
# from PyQt5.QtCore import QStringListModel
from PyQt5 import QtWidgets
# from PyQt5.Qt import *
# from PyQt5 import QtCore, QtGui, QtWidgets
# from PyQt5.QtGui import *
# from PyQt5.QtCore import *

#
# import sys, subprocess, rospy, time
# import serial, binascii
# import paho.mqtt.client as mqtt
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
# import qdarkstyle
import multiprocessing, psutil

import rospy, subprocess
# from std_msgs.msg import *
from geometry_msgs.msg import *


# from PyQt5.QtGui import *






class ListViewDemo(QWidget):
    def __init__(self,parent=None):
        super(ListViewDemo, self).__init__(parent)
        #设置初始大小与标题
        self.resize(300,270)
        self.setWindowTitle('QListView 多选问题')

        #垂直布局
        self.layout=QVBoxLayout()

        #实例化列表视图
        self.listview=QListView()

        #实例化列表模型，添加数据
        self.slm=QStringListModel()
        self.qList=['Item 1','Item 2','Item 3','Item 4','Item 5','Item 6','Item 7','Item 8','Item 9']

        #设置模型列表视图，加载数据列表
        self.slm.setStringList(self.qList)

        #设置列表视图的模型
        self.listview.setModel(self.slm)

        # 多选
        self.listview.setSelectionMode(QAbstractItemView.ExtendedSelection)
        # 不能对表格进行修改（双击重命名等）
        self.listview.setEditTriggers(QAbstractItemView.NoEditTriggers)

        self.label_dqxz = QtWidgets.QLabel()
        self.label_dqxz.setText("当前选择：-")

        #单击触发自定义的槽函数
        self.listview.clicked.connect(self.clicked)



        #设置窗口布局，加载控件
        self.layout.addWidget(self.listview)
        self.layout.addWidget(self.label_dqxz)
        self.setLayout(self.layout)

        camera = subprocess.Popen('roslaunch realsense2_camera rs_camera_vegetable.launch', shell=True)
        rospy.sleep(1.5)

    def clicked(self):
        textlist=''
        for i in self.listview.selectedIndexes():
            text=i.data()
            textlist=textlist+' '+text
        self.label_dqxz.setText('当前选择：'+ str(textlist))


if __name__ == '__main__':
    app=QApplication(sys.argv)
    win=ListViewDemo()
    win.show()
    sys.exit(app.exec_())


