#!/usr/bin/env python
#coding=UTF-8
#-------------------------------------#
#       创建YOLO类
#-------------------------------------#
import colorsys
import os
import time

import numpy as np
import torch, sys
reload(sys)
sys.setdefaultencoding('utf-8')
import torch.nn as nn
from PIL import Image, ImageDraw, ImageFont
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/nets')
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/utils')
from yolo4 import YoloBody
from utils import (DecodeBox, letterbox_image, non_max_suppression,
                         yolo_correct_boxes)


#--------------------------------------------#
#   使用自己训练好的模型预测需要修改2个参数Epoch37-Total_Loss2.4739-Val_Loss2.9858.pth
#   model_path和classes_path都需要修改！
#   如果出现shape不匹配，一定要注意
#   训练时的model_path和classes_path参数的修改
#--------------------------------------------#
class YOLO(object):
    _defaults = {
        # "model_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/Epoch197-Total_Loss0.9522-Val_Loss9.3107.pth',
        # "anchors_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/yolo_anchors.txt',
        # "classes_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/voc_classes.txt',

        # "model_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/Epoch300-Total_Loss4.2894-Val_Loss3.7970.pth',
        # "anchors_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/alldishes_yolo_anchors.txt',
        # "classes_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/alldishes_class.txt',

        # "model_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/Epoch270-Total_Loss0.1613-Val_Loss0.2385.pth',
        # "anchors_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/experiment_dishes_yolo_anchors.txt',
        # "classes_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/experiment_dishes_class.txt',

        "model_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/Epoch186-Total_Loss0.1888-Val_Loss0.0940.pth',
        "anchors_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/experiment_dishes_yolo_anchors.txt',
        "classes_path": '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/experiment_dishes_class.txt',
        "model_image_size"  : (416, 416, 3),
        "confidence"        : 0.5,
        "iou"               : 0.3,
        "cuda"              : False,
        #---------------------------------------------------------------------#
        #   该变量用于控制是否使用letterbox_image对输入图像进行不失真的resize，
        #   在多次测试后，发现关闭letterbox_image直接resize的效果更好
        #---------------------------------------------------------------------#
        "letterbox_image"   : False,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    #---------------------------------------------------#
    #   初始化YOLO
    #---------------------------------------------------#
    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults)
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        # print self.class_names
        # print self.anchors
        # self.class_names = 3
        # self.anchors = 3
        self.generate()

    #---------------------------------------------------#
    #   获得所有的分类
    #---------------------------------------------------#
    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names
    
    #---------------------------------------------------#
    #   获得所有的先验框
    #---------------------------------------------------#
    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape([-1, 3, 2])[::-1,:,:]

    #---------------------------------------------------#
    #   生成模型
    #---------------------------------------------------#
    def generate(self):
        #---------------------------------------------------#
        #   建立yolov4模型
        #---------------------------------------------------#
        self.net = YoloBody(len(self.anchors[0]), len(self.class_names)).eval()
        # self.net = YoloBody(3,3).eval()
        #---------------------------------------------------#
        #   载入yolov4模型的权重
        #---------------------------------------------------#
        print('Loading weights into state dict...')
        device = torch.device('cpu')
        state_dict = torch.load(self.model_path, map_location=device)
        self.net.load_state_dict(state_dict)
        print('Finished!')
        
        # if self.cuda:
        #     self.net = nn.DataParallel(self.net)
        #     self.net = self.net.cuda()

        #---------------------------------------------------#
        #   建立三个特征层解码用的工具
        #---------------------------------------------------#
        self.yolo_decodes = []
        for i in range(3):
            self.yolo_decodes.append(DecodeBox(self.anchors[i], len(self.class_names),  (self.model_image_size[1], self.model_image_size[0])))

        print('{} model, anchors, and classes loaded.'.format(self.model_path))
        # 画框设置不同的颜色
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))

    #---------------------------------------------------#
    #   检测图片
    #---------------------------------------------------#
    def detect_image(self, image):
        #---------------------------------------------------------#
        #   在这里将图像转换成RGB图像，防止灰度图在预测时报错。
        #---------------------------------------------------------#
        # qt 需要
        image = Image.fromarray(image)
        image = image.convert('RGB')

        image_shape = np.array(np.shape(image)[0:2])
        #---------------------------------------------------------#
        #   给图像增加灰条，实现不失真的resize
        #   也可以直接resize进行识别
        #---------------------------------------------------------#
        if self.letterbox_image:
            crop_img = np.array(letterbox_image(image, (self.model_image_size[1],self.model_image_size[0])))
        else:
            crop_img = image.resize((self.model_image_size[1],self.model_image_size[0]), Image.BICUBIC)
        photo = np.array(crop_img,dtype = np.float32) / 255.0
        photo = np.transpose(photo, (2, 0, 1))
        #---------------------------------------------------------#
        #   添加上batch_size维度
        #---------------------------------------------------------#
        images = [photo]

        with torch.no_grad():
            images = torch.from_numpy(np.asarray(images))
            # if self.cuda:
            #     images = images.cuda()

            #---------------------------------------------------------#
            #   将图像输入网络当中进行预测！
            #---------------------------------------------------------#
            outputs = self.net(images)
            output_list = []
            for i in range(3):
                output_list.append(self.yolo_decodes[i](outputs[i]))

            #---------------------------------------------------------#
            #   将预测框进行堆叠，然后进行非极大抑制
            #---------------------------------------------------------#
            output = torch.cat(output_list, 1)
            batch_detections = non_max_suppression(output, len(self.class_names),
                                                    conf_thres=self.confidence,
                                                    nms_thres=self.iou)

            #---------------------------------------------------------#
            #   如果没有检测出物体，返回原图
            #---------------------------------------------------------#
            try:
                batch_detections = batch_detections[0].cpu().numpy()
            except:
                return "None", np.asarray(image)
            
            #---------------------------------------------------------#
            #   对预测框进行得分筛选
            #---------------------------------------------------------#
            top_index = batch_detections[:,4] * batch_detections[:,5] > self.confidence
            top_conf = batch_detections[top_index,4]*batch_detections[top_index,5]
            top_label = np.array(batch_detections[top_index,-1],np.int32)
            top_bboxes = np.array(batch_detections[top_index,:4])
            top_xmin, top_ymin, top_xmax, top_ymax = np.expand_dims(top_bboxes[:,0],-1),np.expand_dims(top_bboxes[:,1],-1),np.expand_dims(top_bboxes[:,2],-1),np.expand_dims(top_bboxes[:,3],-1)

            #-----------------------------------------------------------------#
            #   在图像传入网络预测前会进行letterbox_image给图像周围添加灰条
            #   因此生成的top_bboxes是相对于有灰条的图像的
            #   我们需要对其进行修改，去除灰条的部分。
            #-----------------------------------------------------------------#
            if self.letterbox_image:
                boxes = yolo_correct_boxes(top_ymin,top_xmin,top_ymax,top_xmax,np.array([self.model_image_size[0],self.model_image_size[1]]),image_shape)
            else:
                top_xmin = top_xmin / self.model_image_size[1] * image_shape[1]
                top_ymin = top_ymin / self.model_image_size[0] * image_shape[0]
                top_xmax = top_xmax / self.model_image_size[1] * image_shape[1]
                top_ymax = top_ymax / self.model_image_size[0] * image_shape[0]
                boxes = np.concatenate([top_ymin,top_xmin,top_ymax,top_xmax], axis=-1)
        font_path = '/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts/yolov4_torch_dish/model_data/simsun.ttc'
        font = ImageFont.truetype(font=font_path,size=np.floor(3e-2 * np.shape(image)[1] + 0.5).astype('int32'))

        thickness = max((np.shape(image)[0] + np.shape(image)[1]) // self.model_image_size[0], 1)
        # print top_label
        score = 0
        predicted_class = None
        index = -1
        if len(top_label) != 0:
            for i, c in enumerate(top_label):
                print self.class_names[c], top_conf[i]
                if top_conf[i] > score:
                    score = top_conf[i]
                    predicted_class = self.class_names[c]
                    index = i

        # for i, c in enumerate(top_label):
        if index != -1 and predicted_class != None:

            top, left, bottom, right = boxes[index]
            top = top - 5
            left = left - 5
            bottom = bottom + 5
            right = right + 5

            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(np.shape(image)[0], np.floor(bottom + 0.5).astype('int32'))
            right = min(np.shape(image)[1], np.floor(right + 0.5).astype('int32'))

            # 画框框
            label = u'{} {:.2f}'.format(predicted_class, score)
            # print label

        draw = ImageDraw.Draw(image)
        label_size = draw.textsize(label, font)

        # label = unicode(label, "utf-8")
        # label = label.encode('utf-8')
        # print(label, top, left, bottom, right)

        if top - label_size[1] >= 0:
            text_origin = np.array([left, top - label_size[1]])
        else:
            text_origin = np.array([left, top + 1])

        for i in range(thickness):
            draw.rectangle(
                [left + i, top + i, right - i, bottom - i],
                outline=self.colors[self.class_names.index(predicted_class)])
        draw.rectangle(
            [tuple(text_origin), tuple(text_origin + label_size)],
            fill=self.colors[self.class_names.index(predicted_class)])

        draw.text(text_origin, (label), fill=(0, 0, 0), font=font)
        del draw
        print "result", predicted_class
        return predicted_class, np.asarray(image)

    def get_FPS(self, image, test_interval):
        # 调整图片使其符合输入要求
        image_shape = np.array(np.shape(image)[0:2])

        #---------------------------------------------------------#
        #   给图像增加灰条，实现不失真的resize
        #   也可以直接resize进行识别
        #---------------------------------------------------------#
        if self.letterbox_image:
            crop_img = np.array(letterbox_image(image, (self.model_image_size[1],self.model_image_size[0])))
        else:
            crop_img = image.convert('RGB')
            crop_img = crop_img.resize((self.model_image_size[1],self.model_image_size[0]), Image.BICUBIC)
        photo = np.array(crop_img,dtype = np.float32) / 255.0
        photo = np.transpose(photo, (2, 0, 1))
        #---------------------------------------------------------#
        #   添加上batch_size维度
        #---------------------------------------------------------#
        images = [photo]

        with torch.no_grad():
            images = torch.from_numpy(np.asarray(images))
            if self.cuda:
                images = images.cuda()
            outputs = self.net(images)
            output_list = []
            for i in range(3):
                output_list.append(self.yolo_decodes[i](outputs[i]))
            output = torch.cat(output_list, 1)
            batch_detections = non_max_suppression(output, len(self.class_names),
                                                    conf_thres=self.confidence,
                                                    nms_thres=self.iou)
            try:
                batch_detections = batch_detections[0].cpu().numpy()
                top_index = batch_detections[:,4]*batch_detections[:,5] > self.confidence
                top_conf = batch_detections[top_index,4]*batch_detections[top_index,5]
                top_label = np.array(batch_detections[top_index,-1],np.int32)
                top_bboxes = np.array(batch_detections[top_index,:4])
                top_xmin, top_ymin, top_xmax, top_ymax = np.expand_dims(top_bboxes[:,0],-1),np.expand_dims(top_bboxes[:,1],-1),np.expand_dims(top_bboxes[:,2],-1),np.expand_dims(top_bboxes[:,3],-1)
                
                if self.letterbox_image:
                    boxes = yolo_correct_boxes(top_ymin,top_xmin,top_ymax,top_xmax,np.array([self.model_image_size[0],self.model_image_size[1]]),image_shape)
                else:
                    top_xmin = top_xmin / self.model_image_size[1] * image_shape[1]
                    top_ymin = top_ymin / self.model_image_size[0] * image_shape[0]
                    top_xmax = top_xmax / self.model_image_size[1] * image_shape[1]
                    top_ymax = top_ymax / self.model_image_size[0] * image_shape[0]
                    boxes = np.concatenate([top_ymin,top_xmin,top_ymax,top_xmax], axis=-1)
            
            except:
                pass
                
        t1 = time.time()
        for _ in range(test_interval):
            with torch.no_grad():
                outputs = self.net(images)
                output_list = []
                for i in range(3):
                    output_list.append(self.yolo_decodes[i](outputs[i]))
                output = torch.cat(output_list, 1)
                batch_detections = non_max_suppression(output, len(self.class_names),
                                                        conf_thres=self.confidence,
                                                        nms_thres=self.iou)
                try:
                    batch_detections = batch_detections[0].cpu().numpy()
                    top_index = batch_detections[:,4]*batch_detections[:,5] > self.confidence
                    top_conf = batch_detections[top_index,4]*batch_detections[top_index,5]
                    top_label = np.array(batch_detections[top_index,-1],np.int32)
                    top_bboxes = np.array(batch_detections[top_index,:4])
                    top_xmin, top_ymin, top_xmax, top_ymax = np.expand_dims(top_bboxes[:,0],-1),np.expand_dims(top_bboxes[:,1],-1),np.expand_dims(top_bboxes[:,2],-1),np.expand_dims(top_bboxes[:,3],-1)
                   
                    if self.letterbox_image:
                        boxes = yolo_correct_boxes(top_ymin,top_xmin,top_ymax,top_xmax,np.array([self.model_image_size[0],self.model_image_size[1]]),image_shape)
                    else:
                        top_xmin = top_xmin / self.model_image_size[1] * image_shape[1]
                        top_ymin = top_ymin / self.model_image_size[0] * image_shape[0]
                        top_xmax = top_xmax / self.model_image_size[1] * image_shape[1]
                        top_ymax = top_ymax / self.model_image_size[0] * image_shape[0]
                        boxes = np.concatenate([top_ymin,top_xmin,top_ymax,top_xmax], axis=-1)
                
                except:
                    pass

        t2 = time.time()
        tact_time = (t2 - t1) / test_interval
        return tact_time

