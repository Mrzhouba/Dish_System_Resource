#!/usr/bin/env python
#coding=UTF-8
import rospy
from std_msgs.msg import String
from robot_order.msg import food_msg
import requests
import json
import sys

# reload(sys)
# sys.setdefaultencoding('utf-8')

class acquire_orders_and_feedback():
    def acquire_orders(self):
        # pub = rospy.Publisher('message', food_msg, queue_size=10)

        url = "http://canteen.cailh.club/sell/robot/order?robotId=0&canteen=0"

        result = requests.get(url)
        data = json.loads(result.text)
        # print type(data)
        # print data
        # print(data.keys())
        food = food_msg()
        if data['success']:

            food.id = data['id']

            food.food = json.dumps(data['food'])
            # rospy.loginfo(data)
            # print json.loads(food.food)
            # self.set_order_running(food)
            # pub.publish(food)
            return json.loads(food.food), food.id

        else:
            # print("no order")
            return "no order", 0

    #  单子接收成功反馈
    def set_order_running(self, msg):
        rospy.loginfo("food: id:%s", msg)
        url = "http://canteen.cailh.club/sell/robot/running?id=" + msg
        result = requests.get(url)
        # print result.status_code

    #  单子完成反馈
    def set_order_finish(self, msg):
        rospy.loginfo("food: id:%s", msg)
        url = "http://canteen.cailh.club/sell/robot/finish?id=" + msg
        result = requests.get(url)
        # print result.status_code

if __name__ == '__main__':
    q = acquire_orders_and_feedback()
    foodAndNum, order_id = q.acquire_orders()
    q.set_order_running(order_id)
    print foodAndNum, order_id
    # while(1):
    #     foodAndNum, order_id = q.acquire_orders()
    #     rospy.sleep(2)
    #     print foodAndNum, order_id
    #     dish_num = len(foodAndNum)
    #     bowl_num = 0
    #     dish_list = []
    #
    #     for i in range(dish_num):
    #         bowl_num += foodAndNum[foodAndNum.keys()[i]]
    #         for _ in range(foodAndNum[foodAndNum.keys()[i]]):
    #             dish_list.append(str(foodAndNum.keys()[i]).replace('u\'', '\''))
    #
    #     ordef_list = [bowl_num, dish_list]
    #     print ordef_list

