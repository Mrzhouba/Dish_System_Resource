#!/usr/bin/env python
#coding=UTF-8
import sys
import argparse
import numpy as np
import torch
import torch.nn.parallel
import torch.utils.data
from collections import OrderedDict
sys.path.append('/home/robot/catkin_ws/src/dual_arm_car_grippers_modern_driver/scripts_new/pointnet2_pytorch/models')
from pointnet2_seg import pointnet2_seg_ssg, seg_loss
from pointnet_part_seg import get_model, get_loss
from model_dgcnn import DGCNN_partseg
from model_dgg import dynamic_resnetXt_covariance
from model_dgg_double import model_dgg_double

class get_model_output():
    def __init__(self, model_name, channels, nclasses, weight_path):

        self.model_name = model_name
        #initial model
        self.device = torch.device('cpu')
        if model_name == "pointnet2":
            self.model = pointnet2_seg_ssg(channels, nclasses)
            state_dict = torch.load(weight_path, map_location=self.device)
        elif model_name == "DGCNN":
            self.model = DGCNN_partseg(nclasses)
            multi_state_dict = torch.load(weight_path, map_location=self.device)
            state_dict = self.remove_key_module(multi_state_dict)
        elif model_name == "DGG":
            self.model = dynamic_resnetXt_covariance(nclasses)
            multi_state_dict = torch.load(weight_path, map_location=self.device)
            state_dict = self.remove_key_module(multi_state_dict)
        elif model_name == "DGG-D":
            self.model = model_dgg_double(nclasses)
            multi_state_dict = torch.load(weight_path, map_location=self.device)
            state_dict = self.remove_key_module(multi_state_dict)
        elif model_name == "pointnet":
            self.model = get_model(nclasses, False)
            state_dict = torch.load(weight_path, map_location=self.device)['model_state_dict']

        self.model.load_state_dict(state_dict)
        self.model = self.model.to(self.device)
        self.model.eval()
        print "pointseg model weights loading finish"

    def get_result(self, points_numpy):
        if self.model_name == 'pointnet2':
            dtype = torch.float
            xyz, points = points_numpy[:, :3], points_numpy[:, 3:]
            # print points
            xyz_points = torch.from_numpy(xyz)
            l0_points = torch.from_numpy(points)

            length = points_numpy.shape[1]
            rows = points_numpy.shape[0]
            xyz_points = xyz_points.view(1, rows, 3)
            l0_points = l0_points.view(1, rows, length-3)
            # points = points.transpose(1, 2).contiguous()
            xyz_points = xyz_points.to(self.device, dtype)
            l0_points = l0_points.to(self.device, dtype)
            # points = points.permute(0, 2, 1)

            pred_result = self.model(xyz_points.to(self.device), l0_points.to(self.device))
            pred_choice = pred_result.cpu().max(1)[1]
            result = pred_choice.numpy()[0]

            return result
        elif self.model_name == 'DGCNN':
            data = torch.from_numpy(points_numpy)
            data = data.view(1, data.shape[0], 3)
            data = data.permute(0, 2, 1)
            data = data.type(torch.FloatTensor)
            label_one_hot = np.zeros((1, 16))
            label_one_hot[0, 4] = 1
            label_one_hot = torch.from_numpy(label_one_hot.astype(np.float32))

            seg_pred = self.model(data, label_one_hot)

            seg_pred = seg_pred.permute(0, 2, 1).contiguous()
            pred = seg_pred.max(dim=2)[1]
            # print pred
            return pred[0]

        elif self.model_name == 'DGG' or self.model_name == 'DGG-D':
            data = torch.from_numpy(points_numpy)
            data = data.view(1, data.shape[0], 3)
            data = data.permute(0, 2, 1)
            data = data.type(torch.FloatTensor)
            label_one_hot = np.zeros((1, 16))
            label_one_hot[0, 4] = 1
            label_one_hot = torch.from_numpy(label_one_hot.astype(np.float32))

            seg_pred = self.model(data, label_one_hot)

            seg_pred = seg_pred.permute(0, 2, 1).contiguous()
            pred = seg_pred.max(dim=2)[1]
            return pred[0]

        elif self.model_name =='pointnet':
            data = torch.from_numpy(points_numpy)
            data = data.view(1, data.shape[0], 3)
            data = data.permute(0, 2, 1)
            data = data.type(torch.FloatTensor)
            label_one_hot = np.zeros((1, 7))
            label_one_hot[0, 3] = 1
            label_one_hot = torch.from_numpy(label_one_hot.astype(np.float32))
            # print(1111)
            # print(label_one_hot)
            with torch.no_grad():
                seg_pred, _ = self.model(data, label_one_hot)
            pred = seg_pred.data.cpu().max(2)[1]
            # print(pred)
            return pred[0]

    def remove_key_module(self, state_dict):
        new_state_dict = OrderedDict()
        for k, v in state_dict.items():
            name = k[7:]
            new_state_dict[name] = v
        return new_state_dict

if __name__ == '__main__':
    get_model_output(5,3, '../log/dish_density/checkpoints/pointnet2_seg_300_acc_0.9378.pth')