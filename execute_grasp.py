import argparse
import numpy as np
import open3d as o3d
import os
import scipy.io as scio
import time
import torch
from graspnetAPI import GraspGroup
from PIL import Image

from GraspNetToolBox.config import KINECT_MASK_IAMGE_PATH, REALSENSE_MASK_IAMGE_PATH
# network
from GraspNetToolBox.models.graspnet import GraspNet, pred_decode
from GraspNetToolBox.utils.calibration_helpers import get_intrinsics_matrix
# collision_detector
from GraspNetToolBox.utils.collision_detector import ModelFreeCollisionDetector
# data process tools
from GraspNetToolBox.utils.data_utils import CameraInfo, create_point_cloud_from_depth_image
# gripper control
from GraspNetToolBox.utils.gripper_helpers import GripperController
from GraspNetToolBox.utils.image_helpers import KinectCamera, RealsenseCamera
from GraspNetToolBox.utils.ord_helpers import q_to_matrix, matrix_to_q, ord_camera_to_base, rot_camera_to_q_base, ord_camera_to_hand, ord_hand_to_base
# robot control
from GraspNetToolBox.utils.robot_heplers import RobotController


def parse_args():
    # get args
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, help='Data source')
    parser.add_argument(
        '--move_robot', action='store_true', help='Whether move robot')
    parser.add_argument(
        '--checkpoint_path', required=True, help='Model checkpoint path')
    parser.add_argument(
        '--num_point',
        type=int,
        default=20000,
        help='Point Number [default: 20000]')
    parser.add_argument(
        '--num_view', type=int, default=300, help='View Number [default: 300]')
    parser.add_argument(
        '--collision_thresh',
        type=float,
        default=0.001,
        help='Collision Threshold in collision detection [default: 0.01]')
    parser.add_argument(
        '--voxel_size',
        type=float,
        default=0.005,
        help='Voxel Size to process point clouds' +
        'before collision detection [default: 0.005]')
    parser.add_argument(
        '--approach_dist',
        type=float,
        default=0.1,
        help='approach_dist for collision detect default: 0.1]')
    cfgs = parser.parse_args()
    return cfgs


def get_net():
    # Init the model
    net = GraspNet(
        input_feature_dim=0,
        num_view=cfgs.num_view,
        num_angle=12,
        num_depth=4,
        cylinder_radius=0.05,
        hmin=-0.02,
        hmax_list=[0.01, 0.02, 0.03, 0.04],
        is_training=False)
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    net.to(device)
    # Load checkpoint
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print('-> loaded checkpoint %s (epoch: %d)' %
          (cfgs.checkpoint_path, start_epoch))
    # set model to eval mode
    net.eval()
    return net


def get_and_process_data(camera='Kinect',
                         data_dir=None,
                         color=None,
                         depth=None,
                         cloud=None):
    # get cloud if given cloud is None
    if cloud is None:
        # load data
        if color is None:
            color = np.array(
                Image.open(os.path.join(data_dir, 'color.png')),
                dtype=np.float32) / 255.0
        if depth is None:
            depth = np.array(Image.open(os.path.join(data_dir, 'depth.png')))

        # get intrinsics_matrix from config or file
        if cfgs.source == 'file':
            meta = scio.loadmat(os.path.join(data_dir, 'meta.mat'))
            intrinsic = meta['intrinsic_matrix']
            factor_depth = meta['factor_depth']
        else:
            # kinect, but has some bugs
            intrinsic = get_intrinsics_matrix()
            factor_depth = [[1000.0]]

        # generate cloud
        camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1],
                            intrinsic[0][2], intrinsic[1][2], factor_depth)
        cloud = create_point_cloud_from_depth_image(
            depth, camera, organized=True)
    # get mask
    try:
        if camera == 'kinect':
            workspace_mask = np.array(Image.open(KINECT_MASK_IAMGE_PATH))
        else:
            workspace_mask = np.array(Image.open(REALSENSE_MASK_IAMGE_PATH))
    except Exception:
        workspace_mask = np.ones(np.shape(depth), dtype=bool)

    # get valid points
    mask = (workspace_mask & (depth > 0))
    print(np.shape(mask), np.shape(cloud))
    cloud_masked = cloud[mask]
    color_masked = color[mask]

    # sample points
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(
            len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(
            len(cloud_masked),
            cfgs.num_point - len(cloud_masked),
            replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data to o3d object
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_sampled.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_sampled.astype(np.float32))

    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(
        np.float32))
    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
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
    collision_mask = mfcdetector.detect(
        gg,
        approach_dist=cfgs.approach_dist,
        collision_thresh=cfgs.collision_thresh)
    gg = gg[~collision_mask]
    return gg


def vis_grasps(gg, cloud, count=50):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:count]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])


def get_grasp_from_file(data_dir, show_figure=False):
    net = get_net()
    end_points, cloud = get_and_process_data(data_dir=data_dir)
    gg = get_grasps(net, end_points)

    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    if show_figure:
        vis_grasps(gg, cloud)
    return gg


def get_grasp_from_camera(camera_type='kinect', show_figure=False):
    # get image
    print('-' * 20 + 'starting camera' + '-' * 20)
    if camera_type == 'kinect':
        camera = KinectCamera()
    else:
        camera = RealsenseCamera()
    # camera.show_image()
    image_rgb, image_depth = camera.get_image()
    pointcloud = camera.get_pointcloud()

    print('-' * 20 + 'image get' + '-' * 20)

    # get net
    net = get_net()
    print('-' * 20 + 'network get' + '-' * 20)

    # process data
    end_points, cloud = get_and_process_data(
        camera=camera_type,
        color=image_rgb,
        depth=image_depth,
        cloud=pointcloud)
    print('-' * 20 + 'image processed' + '-' * 20)

    # get grasps
    gg = get_grasps(net, end_points)
    print('-' * 20 + 'collision detecting' + '-' * 20)

    # collision_detection
    if cfgs.collision_thresh > 0:
        gg = collision_detection(gg, np.array(cloud.points))
    print('-' * 20 + 'final grasps get' + '-' * 20)

    if show_figure:
        vis_grasps(gg, cloud)
    return cloud, gg


if __name__ == '__main__':
    # parse args
    cfgs = parse_args()
    print('*' * 100)
    print('using configs:')
    for key in vars(cfgs):
        print(key, ': ', getattr(cfgs, key))
    print('*' * 100)
    # move ur robot to start point, reset gripper
    if cfgs.move_robot is True:
        # start controller
        robot_controller = RobotController()
        gripper_controller = GripperController()
        # reset robot
        robot_controller.reset_robot()
        # activate gripper
        gripper_controller.activate_gripper()
        # open gripper
        gripper_controller.open_gripper()

    # get grasps
    if cfgs.source == 'file':
        data_dir = 'GraspNetToolBox/doc/example_data'
        grasps = get_grasp_from_file(data_dir, show_figure=True)
    else:
        cloud, grasps = get_grasp_from_camera(
            camera_type=cfgs.source, show_figure=False)

    # get grasp ords
    grasps.sort_by_score()
    filtered_grasps = []
    for grasp in grasps:
        ord_in_camera = grasp.translation
        ord_in_base = ord_camera_to_base(cfgs.source, ord_in_camera)
        # filter grasps that is too low
        if ord_in_base[2] > 0.05 and ord_in_base[2] < 0.5:
            filtered_grasps.append(grasp)

    print('grasp count:', len(filtered_grasps))

    # get best grasp
    best_grasp = filtered_grasps[0]
    print(best_grasp)
    best_grasp.
    # execute real grasp if score is high enough
    if best_grasp.score > 0.2 and cfgs.move_robot:
        # ord and rot transform
        ord_in_camera = best_grasp.translation
        rot_in_camera = best_grasp.rotation_matrix

        ord_in_base = ord_camera_to_base(cfgs.source, ord_in_camera)
        q_in_base = rot_camera_to_q_base(rot_in_camera)
        # trans rotation matrix to quaternion
        print('*' * 100)
        print('grasp ord:', ord_in_base)
        print('grasp rot:', q_in_base)
        print('*' * 100)

        # show final result
        gripper = best_grasp.to_open3d_geometry()
        o3d.visualization.draw_geometries([cloud, gripper])

        # wait for confirm
        confirm = input('Input y/n for grasp executing: ')
        if confirm == 'y':
            # move robot pos=ord_in_base,
            robot_controller.move_robot(rotation=q_in_base, v=0.05, a=0.3)
            # close gripper
            gripper_controller.close_gripper()
            # reset robot to starting point
            robot_controller.reset_robot()
            # wait for sometime
            time.sleep(2)
            # open gripper
            gripper_controller.open_gripper()
