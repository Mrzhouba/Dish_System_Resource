#!/usr/bin/env python
#coding=UTF-8

import cv2, time
import numpy as np
import glob
import rospy, sys, tf
import moveit_commander
# import cv2.aruco as aruco
from sensor_msgs.msg import JointState,Image,CameraInfo
from cv_bridge import CvBridge,CvBridgeError
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/recognize_number_test')
from cls_pred import predict_number


predict_number = predict_number()




class recognize_number_label:
    def __init__(self):

        # moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("recognize_number_label", anonymous=True, disable_signals=True)


        self.bridge = CvBridge()
        # self.depth_image = Image()
        # self.camera_info = CameraInfo()
        for _ in range(5):
            image_data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=0.5)
            self.pre_num_result(image_data)

    def pre_num_result(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)
        # image = frame[:, :, [2, 1, 0]]
        # cv2.imwrite("/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/recognize_number/pic.jpg",frame)
        number = predict_number.get_result_number(frame)
        return number


    # def end_label(self):
    #     return self.end




if __name__ == "__main__":
    recognize_number_label()
    # rospy.spin()


