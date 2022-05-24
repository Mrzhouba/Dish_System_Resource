#!/usr/bin/env python
#coding=UTF-8
import tf, sys
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish')
import rospy
import moveit_commander
import roslib; roslib.load_manifest('ur_driver')
import open3d as o3d
import paho.mqtt.client as mqtt
import requests
import json
from sensor_msgs.msg import JointState,Image,CameraInfo
from cv_bridge import CvBridge,CvBridgeError
from scan_dish import scan_dish
scan_dish = scan_dish()
from yolo import YOLO

class generate_menu:
    def __init__(self):
        HOST = "192.168.1.176"
        PORT = 1883
        self.menu_client = mqtt.Client("generate_menu")

        self.yolo_detect = YOLO()
        self.menu_client.connect(HOST, PORT, 60)
        self.menu_client.loop_start()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("generate_menu", anonymous=True, disable_signals=True)
        self.bridge = CvBridge()
        # self.send_memu()
        self.move_recognize_dish(3, 0.35)


    def move_recognize_dish(self, total_num, move_one_distance):
        i = total_num
        dish_location_odom = {}
        while(i):
            scan_dish.car_move(move_one_distance, self.menu_client)
            data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=0.5)
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = frame[:, :, [2, 1, 0]]
            food_name, image_result = self.yolo_detect.detect_image(image)
            dish_location_odom[food_name] = [move_one_distance * (total_num - i + 1)]
            i -= 1
        print dish_location_odom
        with open("/home/robot/photo/memu_weizhi_test.txt", "wb") as f:
            f.write(str(dish_location_odom) + '\n')
        scan_dish.car_move(-total_num * move_one_distance, self.menu_client)
        self.send_memu()



    def send_memu(self):
        fw = open("/home/robot/photo/memu_weizhi_test.txt", "r")
        content = fw.read()
        result = eval(content)
        memu={}
        memu_index = 1
        for key in result:
            memu[str(memu_index)] = key
            memu_index = memu_index + 1
        print memu
        url = "http://canteen.cailh.club/sell/robot/updatefood?canteen=0&port=1"
        res = requests.post(url=url, data=json.dumps(memu))
        fw.close()
        #
        # for line in lines:
        #     result = eval(line)
        #     print result
        #     memu[str(memu_index)]=result.keys()[0]
        #     memu_index = memu_index+1
        # print "menu"
        # print memu
        # url = "http://canteen.cailh.club/sell/robot/updatefood?canteen=0&port=1"
        # res = requests.post(url=url, data=json.dumps(memu))
        # fw.close()


if __name__ == "__main__":
    generate_menu()