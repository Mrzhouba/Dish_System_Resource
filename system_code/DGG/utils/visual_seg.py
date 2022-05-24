from __future__ import print_function
import sys
import open3d as o3d
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import rospy

from show3d_balls import showpoints
import argparse
import numpy as np
import torch
import torch.nn.parallel
import torch.utils.data
from tqdm import tqdm
from torch.autograd import Variable
from torch.utils.data import DataLoader

sys.path.append('/home/robot/PointCloud/pointnet++/Pointnet2.PyTorch-master/models')
sys.path.append('/home/robot/PointCloud/pointnet++/Pointnet2.PyTorch-master/data')
from pointnet2_seg import pointnet2_seg_ssg, seg_loss
from ShapeNet import ShapeNet
import matplotlib.pyplot as plt

import get_model_output


def select_points(point, pred_choice):
    pred = pred_choice.numpy()
    print(pred[0])
    n = len(pred[0])

    result = []
    for i in range(n):
        if(pred[0][i] == 1):
            result.append(point[i])
    result = np.array(result)
    return result

parser = argparse.ArgumentParser()

parser.add_argument('--model', type=str, default='../log/dish/checkpoints/pointnet2_seg_90_acc_0.8634.pth', help='model path')
parser.add_argument('--idx', type=int, default=9, help='model index')
parser.add_argument('--data_root', type=str, default='../shapenetcore_partanno_segmentation_benchmark_v0_normal',
                    required=False, help='Root to the dataset')
parser.add_argument('--class_choice', type=str, default='Dish', help='class choice')
parser.add_argument('--npoints', type=int, default=2500, help='Number of the training points')
parser.add_argument('--is_dish', type=bool, default=True, help='object is dish')
parser.add_argument('--nclasses', type=int, default=2, help='Number of classes')

opt = parser.parse_args()
print(opt)

get_result = get_model_output.get_model_output()

d = ShapeNet(
    data_root=opt.data_root,
    class_choice=opt.class_choice,
    split='test',
    npoints=opt.npoints,
    Dish=opt.is_dish)

idx = opt.idx
dtype = torch.float

print("model %d/%d" % (idx, len(d)))
point, seg = d[idx]


points = torch.from_numpy(point)
label = torch.from_numpy(seg)

cmap = plt.cm.get_cmap("hsv", 10)
cmap = np.array([cmap(i) for i in range(10)])[:, :3]

device = torch.device('cpu')
model = pointnet2_seg_ssg(3, opt.nclasses)

model.load_state_dict(torch.load(opt.model, map_location=device))
model = model.to(device)
model.eval()

n = points.shape[0]

points = points.view(1, n, 3)
points = points.transpose(1, 2).contiguous()
points = points.to(device, dtype)
points = points.permute(0, 2, 1)
# print(points.shape)
l0_points = torch.randn(1, 2500, 0)
pred = model(points.to(device), l0_points.to(device))
pred_choice = pred.cpu().max(1)[1]
print(1)
print(pred_choice.numpy()[0])


# pred_color = cmap[result, :]

pred_color = cmap[pred_choice.numpy()[0], :]
area = select_points(point, pred_choice)


# print(area)
# point_cloud = o3d.geometry.PointCloud()
# point_cloud.points = o3d.utility.Vector3dVector(area)
# o3d.visualization.draw_geometries([point_cloud])

showpoints(point, pred_color)


