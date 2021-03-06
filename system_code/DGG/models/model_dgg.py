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
    if idx is None:
        if dim9 == False:
            idx = knn(x, k=k)
        else:
            idx = knn(x[:, 6:], k=k)
    device = torch.device('cpu')
    idx_base = torch.arange(0, batch_size, device=device).view(-1, 1, 1) * num_points
    idx = idx + idx_base
    idx = idx.view(-1)

    _, num_dims, _ = x.size()

    x = x.transpose(2, 1).contiguous()

    feature = x.view(batch_size * num_points, -1)[idx, :]

    feature = feature.view(batch_size, num_points, k, num_dims)
    x = x.view(batch_size, num_points, 1, num_dims).repeat_interleave(k, 2)
    relative_pos = feature - x
    var = torch.var(relative_pos, dim=2)
    var = torch.sub(1, var).view(batch_size, num_points, 1, -1).repeat_interleave(k, 2)

    relative_feature = torch.mul(relative_pos, var)
    relative_feature = torch.cat((x, relative_feature), dim=3).permute(0,3,1,2).contiguous()

    mean = torch.mean(relative_pos, dim=0)
    normalize = relative_pos - mean
    covariance_matrix = torch.matmul(relative_pos, normalize.permute(0,1,3,2))
    covariance_feature = torch.cat((x, covariance_matrix), dim=3).permute(0, 3, 1, 2).contiguous()
    return relative_feature, covariance_feature



class dynamic_resnetXt_covariance(nn.Module):
    def __init__(self, seg_num_all):
        super(dynamic_resnetXt_covariance, self).__init__()
        # self.args = args
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

        self.conv3 = nn.Sequential(nn.Conv2d(6, 64, kernel_size=1, bias=False),
                                   self.bn3,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv4 = nn.Sequential(nn.Conv2d(64, 64, kernel_size=1, bias=False),
                                   self.bn4,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv5 = nn.Sequential(nn.Conv2d(3 + self.k, 64, kernel_size=1, bias=False),
                                   self.bn5,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv6 = nn.Sequential(nn.Conv2d(64, 64, kernel_size=1, bias=False),
                                   self.bn6,
                                   nn.LeakyReLU(negative_slope=0.2))


        self.encode_1 = nn.Sequential(
                                    nn.Conv1d(128, 128, kernel_size=1, bias=False),
                                    nn.BatchNorm1d(128),
                                    nn.LeakyReLU(negative_slope=0.2),
                                    nn.Conv1d(128, 128, kernel_size=1, bias=False),
                                    nn.BatchNorm1d(128),
                                    nn.LeakyReLU(negative_slope=0.2),
                                     )

        self.encode_2 = nn.Sequential(
                                   nn.Conv1d(256, 256, kernel_size=1, bias=False),
                                   nn.BatchNorm1d(256),
                                   nn.LeakyReLU(negative_slope=0.2),
                                   nn.Conv1d(256, 256, kernel_size=1, bias=False),
                                   nn.BatchNorm1d(256),
                                   nn.LeakyReLU(negative_slope=0.2),
                                   )

        self.encode_3 = nn.Sequential(
                                   nn.Conv1d(512, 512, kernel_size=1, bias=False),
                                   nn.BatchNorm1d(512),
                                   nn.LeakyReLU(negative_slope=0.2),
                                      )

        self.conv7 = nn.Sequential(nn.Conv1d(512, 1024, kernel_size=1, bias=False),
                                   self.bn7,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv8 = nn.Sequential(nn.Conv1d(16, 64, kernel_size=1, bias=False),
                                   self.bn8,
                                   nn.LeakyReLU(negative_slope=0.2))
        self.conv9 = nn.Sequential(nn.Conv1d(1088+512, 512, kernel_size=1, bias=False),
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

        realtive, covariance = get_graph_dynamic_covariance(x, k=self.k)

        realtive = self.conv3(realtive)
        realtive = self.conv4(realtive)
        realtive_ = realtive.max(dim=-1, keepdim=False)[0]
        covariance = self.conv5(covariance)
        covariance = self.conv6(covariance)
        covariance_ = covariance.max(dim=-1, keepdim=False)[0]

        all_feature = torch.cat((realtive_, covariance_), dim=1)
        all_feature_1 = self.encode_1(all_feature)
        all_feature_1 = torch.cat((all_feature, all_feature_1), dim=1)
        all_feature_2 = self.encode_2(all_feature_1)
        all_feature_2 = torch.cat((all_feature_1, all_feature_2), dim=1)
        encode_feature = self.encode_3(all_feature_2)

        x = self.conv7(encode_feature)
        x = x.max(dim=-1, keepdim=True)[0]

        l = l.view(batch_size, -1, 1)
        l = self.conv8(l)

        x = torch.cat((x, l), dim=1)
        x = x.repeat(1, 1, num_points)

        x = torch.cat((x, encode_feature),
                      dim=1)

        x = self.conv9(x)
        x = self.dp1(x)
        x = self.conv10(x)
        x = self.dp2(x)
        x = self.conv11(x)
        x = self.conv12(x)

        return x
