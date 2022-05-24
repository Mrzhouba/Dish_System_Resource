#!/usr/bin/env python
#coding=UTF-8
import tf, sys
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish')
import rospy
import moveit_commander
# import roslib; roslib.load_manifest('ur_driver')
import open3d as o3d
import paho.mqtt.client as mqtt
import requests
import json
from sensor_msgs.msg import JointState,Image,CameraInfo
from cv_bridge import CvBridge,CvBridgeError
import scipy.misc
import cv2

from yolo import YOLO
class prdeict():
    def __init__(self):
        self.yolo_detect = YOLO()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("predict", anonymous=True, disable_signals=True)
        self.bridge = CvBridge()
        data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=0.5)

        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image = frame[:, :, [2, 1, 0]]
        cv2.imwrite('/home/robot/test3.png', frame)
        food_name, image_result = self.yolo_detect.detect_image(image)
        image = scipy.misc.toimage(image_result, cmin=0.0, cmax=1.0)
        # image = Image.fromarray(image_result)
        image.show()
        print food_name


if __name__ == "__main__":
    prdeict()