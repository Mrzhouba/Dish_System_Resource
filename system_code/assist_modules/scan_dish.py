#!/usr/bin/env python
#coding=UTF-8
import rospy
import roslib
import requests
import json
import paho.mqtt.client as mqtt
# roslib.load_manifest('ur_driver')
# roslib.load_manifest('dh_hand_driver')
from robot_order.msg import food_msg
import requests
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from io import *
import math



class scan_dish:
    def read_current_loc(self, mqtt_client):
        mqtt_client.subscribe("/distance_car_real", 0)
        mqtt_client.on_message = self.current_loc_message_come
        rospy.sleep(1)

    def current_loc_message_come(self, mqtt_client, userdata, msg):
        str1 = msg.payload.split("\r\n")
        self.current_loc = float(str1[1])
        print self.current_loc

    def send_memu(self):
        fw = open("/home/robot/photo/memu_weizhi_test1.txt", "r+")
        lines = fw.readlines()
        memu={}
        memu_index=1
        for line in lines:
            result = eval(line)
            memu[str(memu_index)]=result.keys()[0]
            memu_index = memu_index+1
        print memu
        url = "http://canteen.cailh.club/sell/robot/updatefood?canteen=0&port=1"
        res = requests.post(url=url, data=json.dumps(memu))
        fw.close()

    def car_set_zero(self,mqtt_client):
        try:
            # 车Home归位
            mqtt_client.publish("/car_steTo_zero",10)
        except:
            print ("The Car initialization failed!")
    # 控制小车运动, move_distance为小车前进点距离, compare_distance为小车已经前进点距离
    def car_move(self,move_distance, mqtt_client):
        # compare_distance = 0

        # cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rate = 50
        # r = rospy.Rate(rate)
        linear_speed = 0.5
        goal_distance = move_distance
        self.real_single = 0

        print ("Car move ready to go: " + str(goal_distance) + " meters!")

        code_transform = str(int(goal_distance/math.pi/0.2/1.2*25*180))
        # mqtt_client.connect(HOST, PORT, 60)
        # print code_transform
        mqtt_client.publish("/car_go","car_pos," + code_transform)
        # self.sleep_signal = True
        # rospy.sleep(0.1)
        # self.sleep_signal = False
        # self.real_single = 0
        rospy.sleep(0.3)
        print "运动距离已发布"

        # linear_duration = abs(goal_distance) / linear_speed
        # move_cmd = Twist()
        #
        # if goal_distance >=  0:
        #     move_cmd.linear.x = linear_speed
        # else:
        #     move_cmd.linear.x = -linear_speed
        # ticks = int(linear_duration * rate)
        # for t in range(ticks):
        #     cmd_vel.publish(move_cmd)
        #     r.sleep()
        # cmd_vel.publish(Twist())

        self.on_subscribe(mqtt_client)
        self.mqtt_compare()
        print ('Move over!\nThe car has been moved ' + str(goal_distance) + ' meters!')

    def on_subscribe(self, mqtt_client):

        mqtt_client.subscribe("/Car_move_complied", 0)
        mqtt_client.on_message = self.Car_message_come

    def on_unsubscribe(self, mqtt_client):
        print "u"

    def Car_message_come(self,mqtt_client,userdata,msg):
        # if self.sleep_signal == True:
        #     rospy.sleep(0.1)
        self.real_single = msg.payload.split("\r\n")[0]
        # print self.real_single

    def call_back(self,data):
        self.real_move = data.position[0]


    def mqtt_compare(self):
        i = 0
        while not (self.real_single== "1.0000"):
            # print self.real_single
            continue

        # print ('ok')


if __name__ == '__main__':
    # import moveit_commander,sys

    # moveit_commander.roscpp_initialize(sys.argv)
    #
    # rospy.init_node("catch_bowl", anonymous=True, disable_signals=True)
    HOST = "192.168.20.176"
    PORT = 1883
    mqtt_client = mqtt.Client("car-jjk")
    mqtt_client.connect(HOST, PORT, 60)
    mqtt_client.loop_start()

    x = scan_dish()
    # x.read_current_loc(mqtt_client)
    # x.on_subscribe(mqtt_client)
    # rospy.sleep(100)
    # x.car_set_zero(mqtt_client)
    # for i in range(10):
    #     x.car_move(-0.35, mqtt_client)
    #     print 1
    #     rospy.sleep(0.5)
    #     x.car_move(0.35, mqtt_client)
    #     rospy.sleep(0.5)
    x.car_move(-0.36, mqtt_client)
    print "end"


      
