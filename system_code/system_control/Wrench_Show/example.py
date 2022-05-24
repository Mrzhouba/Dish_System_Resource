# -*- coding: utf-8 -*-
# import MySQLdb
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QFrame, QHBoxLayout
from pyqtgraph.Qt import QtGui, QtCore
# import pandas as pd

win = pg.GraphicsWindow()
win.setWindowTitle('pyqtgraph example: Scrolling Plots')
p4 = win.addPlot()
p4.setDownsampling(mode='peak')
p4.setClipToView(True)
label = pg.LabelItem(justify='left')
win.addItem(label)


def MySql_data():  # 数据库数据的读取
    conn = MySQLdb.Connect(
        host='127.0.0.1',
        port=3306,
        user='root',
        passwd='123456',
        db='imooc',
        charset='utf8'
    )  # 创建连接
    cursor = conn.cursor()  # 获取cursor
    sql = "select * from mysql_data2  "  # 编写sql语句
    cursor.execute(sql)  # 使用cursor.execute的放法进行处理
    data = cursor.fetchall()  # 将存储在缓冲区的数据拿出
    df = pd.DataFrame(list(data), columns=['licheng', 'data1', 'data2'])
    return df
# df = MySql_data()

win = pg.GraphicsWindow()
win.setWindowTitle('pyqtgraph example: Scrolling Plots')
p4 = win.addPlot()
p4.setDownsampling(mode='peak')
p4.setClipToView(True)
label = pg.LabelItem(justify='left')
win.addItem(label)
# 添加折线图的标题
data_list = []
curve4 = p4.plot([0])
# data3 = array.array('d')
# data_x = array.array('d')
ptr3 = 0


def mouseMoved(evt):  # 鼠标显示坐标
    mousePoint = p4.vb.mapSceneToView(evt[0])
    label.setText(
        "<span style='font-size: 14pt; color: white'> x = %0.2f, <span style='color: white'> y = %0.2f</span>" % (
            mousePoint.x(), mousePoint.y()))

def update2():
    data_list.append(1)
    curve4.setData(data_list)
    # global data3, ptr3
    # tmp = df['licheng'][ptr3]   #获取数据库中licheng数据 ---用来设置x值
    # tmp1 = df['data1'][ptr3]    #获取到数据库中data1的数据 ---用来设置y值
    #
    # if ptr3 <= 100:
    #     data3.append(tmp1)
    #     data_x.append(tmp)
    #     curve4.setData(data_x[:ptr3],data3[:ptr3])   #设置x轴y轴值
    # else:
    #     data3[:-1] = data3[1:]
    #     data3[-1] = tmp1
    #     data_x[:-1] = data_x[1:]
    #     data_x[-1] = tmp
    #     curve4.setData(data_x,np.frombuffer(data3, dtype=np.double))  #设置x轴，y轴的值
    #
    #     # curve4.setPos(ptr3,0)    #设置
    # ptr3 += 1

proxy = pg.SignalProxy(p4.scene().sigMouseMoved, rateLimit=60, slot=mouseMoved)  #调用鼠标函数，实现鼠标数据显示
timer = pg.QtCore.QTimer()
timer.timeout.connect(update2)
timer.start(200)
# from PyQt5.QtGui import QPixmap, QPainter
# from PyQt5.QtWidgets import *
# from PyQt5.QtCore import QRectF
# rect = QGraphicsView.viewport(win.gvPointRecords).rect()
# pixmap = QPixmap(rect.size())
# painter = QPainter(pixmap)
# painter.begin(pixmap)
# p4.gvPointRecords.render(painter, QRectF(pixmap.rect()), rect)
# painter.end()
#
# imgFile, _ = QFileDialog.getSaveFileName(p4, "保存图像", './', "tr (*.png)")
# if imgFile:
#     img = pixmap.save(imgFile)
#     imgFile = imgFile
import pyqtgraph.exporters
ex = pg.exporters.ImageExporter(win.scene())
ex.export(fileName="test.png")


if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
