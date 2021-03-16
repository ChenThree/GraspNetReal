""" Demo to show prediction results.
    Author: chenxi-wang
"""

import os
import sys
import numpy as np
import open3d as o3d
import argparse
import importlib
import scipy.io as scio
from PIL import Image
from matplotlib import pyplot as plt

import torch
from graspnetAPI import GraspGroup

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
sys.path.append(os.path.join(ROOT_DIR, 'dataset'))
sys.path.append(os.path.join(ROOT_DIR, 'utils'))

from models.graspnet import GraspNet, pred_decode
from dataset.graspnet_dataset import GraspNetDataset
from utils.collision_detector import ModelFreeCollisionDetector
from utils.data_utils import CameraInfo, create_point_cloud_from_depth_image
from ord_helpers import base_to_camera, camera_to_base
from calibration_helpers import get_intrinsics_matrix


parser = argparse.ArgumentParser()
parser.add_argument('--source',
                    required=True,
                    help='Data source')
parser.add_argument('--checkpoint_path',
                    required=True,
                    help='Model checkpoint path')
parser.add_argument('--num_point',
                    type=int,
                    default=20000,
                    help='Point Number [default: 20000]')
parser.add_argument('--num_view',
                    type=int,
                    default=300,
                    help='View Number [default: 300]')
parser.add_argument(
    '--collision_thresh',
    type=float,
    default=0.01,
    help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument(
    '--voxel_size',
    type=float,
    default=0.01,
    help=
    'Voxel Size to process point clouds before collision detection [default: 0.01]'
)
cfgs = parser.parse_args()


def get_net():
    # Init the model
    net = GraspNet(input_feature_dim=0,
                   num_view=cfgs.num_view,
                   num_angle=12,
                   num_depth=4,
                   cylinder_radius=0.05,
                   hmin=-0.02,
                   hmax_list=[0.01, 0.02, 0.03, 0.04],
                   is_training=False)
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> loaded checkpoint %s (epoch: %d)" %
          (cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net


def get_and_process_data(data_dir=None, color=None, depth=None, cloud=None):
    # get cloud
    if cloud is None:
        # load data
        if color is None:
            color = np.array(Image.open(os.path.join(data_dir, 'color.png')),
                             dtype=np.float32) / 255.0
        if depth is None:
            depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))

        # 读取内参矩阵或预先给定
        if cfgs.source == 'file':
            meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
            intrinsic = meta['intrinsic_matrix']
            factor_depth = meta['factor_depth']
        else:
            intrinsic = get_intrinsics_matrix()
            factor_depth = [[1000]]

        # generate cloud
        camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1],
                            intrinsic[0][2], intrinsic[1][2], factor_depth)
        cloud = create_point_cloud_from_depth_image(depth,
                                                    camera,
                                                    organized=True)
    # get mask
    try:
        workspace_mask = np.array(
            Image.open('./utils/mask_1.png'))
    except Exception:
        workspace_mask = np.ones(np.shape(depth), dtype=bool)

    # get valid points
    mask = (workspace_mask & (depth > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked),
                                cfgs.num_point,
                                replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked),
                                 cfgs.num_point - len(cloud_masked),
                                 replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(
        np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud


def get_grasps(net, end_points):
    # Forward pass
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg


def collision_detection(gg, cloud):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=cfgs.voxel_size)
    collision_mask = mfcdetector.detect(gg,
                                        approach_dist=0.05,
                                        collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg


def vis_grasps(gg, cloud, count=10):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:count]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])


def demo(data_dir, show_figure=False):
    net = get_net()
    end_points, cloud = get_and_process_data(data_dir)
    gg = get_grasps(net, end_points)
    print(np.shape(cloud.points))
    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    if show_figure:
        vis_grasps(gg, cloud, count=1)
    return gg


def demo_camera(show_figure=False):
    # get image
    print('-' * 20 + 'starting camera' + '-' * 20)
    camera = KinectCamera()
    # camera.show_image()
    image_rgb, image_depth = camera.get_image()
    pointcloud = camera.get_pointcloud()
    print('-' * 20 + 'image get' + '-' * 20)

    # get net
    net = get_net()
    print('-' * 20 + 'network get' + '-' * 20)

    # process data
    end_points, cloud = get_and_process_data(color=image_rgb / 255.0,
                                             depth=image_depth)
    print('-' * 20 + 'image processed' + '-' * 20)

    # get grasps
    gg = get_grasps(net, end_points)
    print('-' * 20 + 'collision detecting' + '-' * 20)

    # collision_detection
    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    print('-' * 20 + 'final grasps get' + '-' * 20)

    if show_figure:
        vis_grasps(gg, cloud, count=20)
    return gg


if __name__ == '__main__':
    # get grasps
    if cfgs.source == 'file':
        data_dir = 'doc/example_data'
        grasps = demo(data_dir, show_figure=False)
    elif cfgs.source == 'camera':
        from image_helpers import KinectCamera
        grasps = demo_camera(show_figure=True)

    # get ord
    grasps.sort_by_score()
    final_grasp = grasps[0]

    # ord transform
    ord_in_camera = final_grasp.translation
    print(ord_in_camera)

    ord_in_base = camera_to_base(ord_in_camera)
    print(ord_in_base)

