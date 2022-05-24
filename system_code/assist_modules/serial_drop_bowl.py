# -*- coding: utf-8 -*-
import serial 
import binascii
import paho.mqtt.client as mqtt
import math, rospy


# # try:
#     # 车Home归位
# mqtt_client.publish("/drop_bowl", 1)
# drop = mqtt_client.subscribe("/drop_complete")
# print drop[0]

class drop_bowl:


    def send_order_drop_bowl(self, bowl_drop_mqtt_client):
        rospy.sleep(0.1)
        self.car_feedback = 0
        bowl_drop_mqtt_client.publish("/drop_bowl", 1)
        self.on_subscribe(bowl_drop_mqtt_client)
        self.mqtt_compare()
        return True

    def on_subscribe(self, client):
        client.subscribe("/drop_complete", 2)
        client.on_message = self.on_message_come
    def on_message_come(self, mqtt_client,userdata, msg):

        self.car_feedback = msg.payload.split("\r\n")[0]

    def mqtt_compare(self):
        i = 0
        while not (self.car_feedback == "01"):
            # print self.real_single
            continue


    def control_convey_belt(self, distance, mqtt_client, is_block):
        # rospy.sleep(0.1)
        self.convey_feedback = 0
        trans_distance = str(int(distance/math.pi/0.05/17*18*16*160))
        print ("Convey move ready to go: " + str(distance) + " meters!")
        mqtt_client.publish("/convey_go", "convey_pos," + trans_distance)
        # print "succ"
        rospy.sleep(0.2)
        if is_block == True:
            self.convey_on_subscribe(mqtt_client)
            self.convey_mqtt_compare()

    def convey_on_subscribe(self, client):
        client.subscribe("/Convey_move_complete", 2)
        client.on_message = self.convey_on_message_come

    def convey_on_message_come(self, mqtt_client,userdata, msg):
        self.convey_feedback = msg.payload.split("\r\n")[0]

    def convey_mqtt_compare(self):
        i = 0
        while not (self.convey_feedback == "1.0000"):
            # print self.real_single
            continue




#
if __name__ == '__main__':
    HOST = "192.168.20.176"
    PORT = 1883
    mqtt_client = mqtt.Client("drop-bowl-convey-control")
    mqtt_client.connect(HOST, PORT, 60)
    mqtt_client.loop_start()
    x = drop_bowl()

    import time
    start = time.time()
    x.control_convey_belt(-1.44, mqtt_client, True)
    print time.time() - start
    # rospy.sleep(2)
    # x.control_convey_belt(-1.5, mqtt_client, True)
    # x.control_convey_belt(0.7, mqtt_client, True)
    # x.control_convey_belt(0.8, mqtt_client, True)
    # for _ in range(10):
    #     res = x.send_order_drop_bowl(mqtt_client)
    #     #
    #     print res