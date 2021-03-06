import os
import sys
import copy
import math
import numpy as np
import torch
import torch.nn as nn
import torch.nn.init as init
import torch.nn.functional as F


def knn(x, k):
    inner = -2 * torch.matmul(x.transpose(2, 1), x)
    xx = torch.sum(x ** 2, dim=1, keepdim=True)
    pairwise_distance = -xx - inner - xx.transpose(2, 1)

    idx = pairwise_distance.topk(k=k, dim=-1)[1]  # (batch_size, num_points, k)
    return idx

def get_graph_dynamic_covariance(x, k=20, idx=None, dim9=False):
    batch_size = x.size(0)
    num_points = x.size(2)
    fea_num = x.size(1)
    initial_x = x

    x = x.view(batch_size, -1, num_points)
    # print(x.shape)
    if idx is None:
        if dim9 == False:
            idx = knn(x, k=k)  # (batch_size, num_points, k)
        else:
            idx = knn(x[:, 6:], k=k)
    device = torch.device('cpu')
    # print("idx:{}".format(idx.shape))
    idx_base = torch.arange(0, batch_size, device=device).view(-1, 1, 1) * num_points
    # print(idx_base)
    # print(idx.shape)
    idx = idx + idx_base
    idx = idx.view(-1)
    # print(idx)

    _, num_dims, _ = x.size()

    x = x.transpose(2, 1).contiguous()  # (batch_size, num_points, num_dims)  -> (batch_size*num_points, num_dims) #   batch_size * num_points * k + range(0, batch_size*num_points)

    feature = x.view(batch_size * num_points, -1)[idx, :]

    feature = feature.view(batch_size, num_points, k, num_dims)
    x = x.view(batch_size, num_points, 1, num_dims).repeat_interleave(k, 2)
    relative_pos = feature - x
    # print(relative_pos)
    # relative_feature = torch.cat((x, relative_pos), dim=3).permute(0,3,1,2).contiguous()
    var = torch.var(relative_pos, dim=2)
    # print(var)
    var = torch.sub(1, var).view(batch_size, num_points, 1, -1).repeat_interleave(k, 2)
    # print(var)

    relative_feature = torch.mul(relative_pos, var)
    relative_feature = torch.cat((x, relative_feature), dim=3)
    # print(relative_feature.shape)

    mean = torch.mean(relative_pos, dim=0)
    normalize = relative_pos - mean
    covariance_matrix = torch.matmul(relative_pos, normalize.permute(0,1,3,2))
    relative_covariance_feature = torch.cat((relative_feature, covariance_matrix), dim=3).permute(0, 3, 1, 2).contiguous()
    # print(covariance_matrix.shape)

    return relative_covariance_feature  # (batch_size, 2*num_dims, num_points, k)



class model_dgg_double(nn.Module):
    def __init__(self, seg_num_all):
        super(model_dgg_double, self).__init__()

        self.seg_num_all = seg_num_all
        self.k = 40

        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(64)
        self.bn3 = nn.BatchNorm2d(64)
        self.bn4 = nn.BatchNorm2d(64)
        self.bn5 = nn.BatchNorm2d(64)
        self.bn6 = nn.BatchNorm2d(64)

        self.bn7 = nn.BatchNorm1d(1024)
        self.bn8 = nn.BatchNorm1d(64)
        self.bn9 = nn.BatchNorm1d(512)
        self.bn10 = nn.BatchNorm1d(256)
        self.bn11 = nn.BatchNorm1d(128)

        self.conv3 = nn.Sequential(nn.Conv2d(46, 64, kernel_size=1, bias=False),
                                   self.bn3,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv4 = nn.Sequential(nn.Conv2d(64, 64, kernel_size=1, bias=False),
                                   self.bn4,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv5 = nn.Sequential(nn.Conv2d(64*2+40, 64, kernel_size=1, bias=False),
                                   self.bn5,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv6 = nn.Sequential(nn.Conv2d(64, 64, kernel_size=1, bias=False),
                                   self.bn6,
                                   nn.LeakyReLU(negative_slope=0.2))


        self.conv7 = nn.Sequential(nn.Conv1d(128, 1024, kernel_size=1, bias=False),
                                   self.bn7,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv8 = nn.Sequential(nn.Conv1d(16, 64, kernel_size=1, bias=False),
                                   self.bn8,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv9 = nn.Sequential(nn.Conv1d(1216, 512, kernel_size=1, bias=False),
                                   self.bn9,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.dp1 = nn.Dropout(p=0.5)
        self.conv10 = nn.Sequential(nn.Conv1d(512, 256, kernel_size=1, bias=False),
                                    self.bn10,
                                    nn.LeakyReLU(negative_slope=0.2))
        self.dp2 = nn.Dropout(p=0.5)
        self.conv11 = nn.Sequential(nn.Conv1d(256, 128, kernel_size=1, bias=False),
                                    self.bn11,
                                    nn.LeakyReLU(negative_slope=0.2))
        self.conv12 = nn.Conv1d(128, self.seg_num_all, kernel_size=1, bias=False)

    def forward(self, x, l):
        batch_size = x.size(0)
        num_points = x.size(2)

        realtive_covariance = get_graph_dynamic_covariance(x, k=self.k)  # (batch_size, 3, num_points) -> (batch_size, 3*2, num_points, k)
        # print(realtive_covariance.shape)
        realtive_covariance = self.conv3(realtive_covariance)  # (batch_size, 64*2, num_points, k) -> (batch_size, 64, num_points, k)
        realtive_covariance = self.conv4(realtive_covariance)  # (batch_size, 64, num_points, k) -> (batch_size, 64, num_points, k)
        realtive_covariance_ = realtive_covariance.max(dim=-1, keepdim=False)[0]  # (batch_size, 64, num_points, k) -> (batch_size, 64, num_points)

        double_realtive_covariance = get_graph_dynamic_covariance(realtive_covariance_, self.k)
        double_realtive_covariance = self.conv5(double_realtive_covariance)
        double_realtive_covariance = self.conv6(double_realtive_covariance)
        double_realtive_covariance_ = double_realtive_covariance.max(dim=-1, keepdim=False)[0]

        all_feature = torch.cat((realtive_covariance_, double_realtive_covariance_), dim=1)

        x = self.conv7(all_feature)  # (batch_size, 64*3, num_points) -> (batch_size, emb_dims, num_points)
        x = x.max(dim=-1, keepdim=True)[0]  # (batch_size, emb_dims, num_points) -> (batch_size, emb_dims, 1)

        l = l.view(batch_size, -1, 1)  # (batch_size, num_categoties, 1)
        l = self.conv8(l)  # (batch_size, num_categoties, 1) -> (batch_size, 64, 1)

        x = torch.cat((x, l), dim=1)  # (batch_size, 1088, 1)
        x = x.repeat(1, 1, num_points)  # (batch_size, 1088, num_points)

        x = torch.cat((x, realtive_covariance_, double_realtive_covariance_),
                      dim=1)  # (batch_size, 1088+64*3, num_points)

        x = self.conv9(x)  # (batch_size, 1088+64*3, num_points) -> (batch_size, 256, num_points)
        x = self.dp1(x)
        x = self.conv10(x)  # (batch_size, 256, num_points) -> (batch_size, 256, num_points)
        x = self.dp2(x)
        x = self.conv11(x)  # (batch_size, 256, num_points) -> (batch_size, 128, num_points)
        x = self.conv12(x)  # (batch_size, 256, num_points) -> (batch_size, seg_num_all, num_points)

        return x
