#!/usr/bin/env python
#coding=UTF-8

import cv2, time
import numpy as np
import glob
import rospy, sys, tf
import moveit_commander
import cv2.aruco as aruco
from sensor_msgs.msg import JointState,Image,CameraInfo
from cv_bridge import CvBridge,CvBridgeError
from geometry_msgs.msg import Pose, PoseStamped, Vector3Stamped


class recognize_marker_trash:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("recognize_marker", anonymous=True, disable_signals=True)
        self.bridge = CvBridge()
        self.depth_image = Image()
        self.camera_info = CameraInfo()
        self.start_loc = 0

    def get_start_loc(self):
        # self.image_sub = rospy.Subscriber("/camera2/color/image_raw", Image, self.callback)
        data = rospy.wait_for_message("/camera2/color/image_raw", Image, timeout=1)
        self.callback(data)
        # self.image_depth_sub = rospy.Subscriber("/camera2/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        # self.camera_info_sub = rospy.Subscriber("/camera2/aligned_depth_to_color/camera_info", CameraInfo,
        #                                         self.info_callback)
        rospy.sleep(0.15)
        # print self.start_loc
        # return self.start_loc

    def depth_callback(self,data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

    def info_callback(self,data):
        self.camera_info = data

    def location(self, image, x, y):
        # 先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
        # print self.depth_image
        real_z = 0.001 * self.depth_image[y,x]
        real_x = (x - self.camera_info.K[2]) / self.camera_info.K[0] * real_z
        real_y = (y - self.camera_info.K[5]) / self.camera_info.K[4] * real_z
        loc =  [real_x, real_y, real_z]
        start_loc_point = PoseStamped()
        start_loc_point.header.frame_id = 'camera2_color_optical_frame'
        start_loc_point.pose.position.x = real_x
        start_loc_point.pose.position.y = real_y
        start_loc_point.pose.position.z = real_z
        start_loc_point.pose.orientation.x = 0
        start_loc_point.pose.orientation.y = 0
        start_loc_point.pose.orientation.z = 0
        start_loc_point.pose.orientation.w = 0

        trans_camera_body = tf.TransformListener()
        trans_camera_body.waitForTransform("/body_link", "/camera2_color_optical_frame", rospy.Time(), rospy.Duration(0.1))

        start_loc_body_link_mess = trans_camera_body.transformPose('/body_link', start_loc_point)

        self.start_loc = start_loc_body_link_mess.pose.position.x
        print self.start_loc
        self.image_sub.unregister()
        self.image_depth_sub.unregister()
        self.camera_info_sub.unregister()

    def callback(self, data):

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray= cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

        aruco_id_to_food = {587: 'RouPiHuangDou', 583: 'JiMiHua', 584: 'ChaoPuGua', 585: 'ZhaTuDou',
                            586: 'JiDanRouPian'}
        food_name = None
        print ids
        if ids != None:
            food_name = aruco_id_to_food[int(ids[0])]
        print food_name
        return food_name



        # if len(corners) != 0:
        #     # print corners
        #     pixel_loc = corners[0][0][0]
        #     # print pixel_loc
        #     self.location(frame, int(pixel_loc[0]), int(pixel_loc[1]))

        # if ids is not None:
        #     rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
        #     (rvec - tvec).any()  # get rid of that nasty numpy value array error
        #
        #     for i in range(rvec.shape[0]):
        #         aruco.drawAxis(frame, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
        #         aruco.drawDetectedMarkers(frame, corners)
        #     cv2.putText(frame, "Id: " + str(ids), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        #
        # else:
        #     cv2.putText(frame, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        # cv2.imshow("frame", frame)
        # key = cv2.waitKey(1)
        # if key == 27:  # 按esc键退出
        #     print('esc break...')
        #     cv2.destroyAllWindows()
        #
        # if key == ord(' '):  # 按空格键保存
        #     #        num = num + 1
        #     #        filename = "frames_%s.jpg" % num  # 保存一张图像
        #     filename = str(time.time())[:10] + ".jpg"
        #     cv2.imwrite(filename, frame)


class recognize_marker:
    def __init__(self):
        # moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("recognize_marker", anonymous=True, disable_signals=True)
        self.bridge = CvBridge()
        self.depth_image = Image()
        self.camera_info = CameraInfo()
        self.start_loc = 0

    def get_location(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("recognize_marker", anonymous=True, disable_signals=True)
        initial_loc = -0.153801992536
        data = rospy.wait_for_message('/aruco_single/position', Vector3Stamped, timeout=5)
        print data.vector

        move_dis = data.vector.y - initial_loc
        print move_dis / 2




if __name__ == "__main__":
    x = recognize_marker_trash()
    x.get_start_loc()


