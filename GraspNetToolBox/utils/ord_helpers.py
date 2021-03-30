import numpy as np
import math

# get q and x,y,z from config
from scipy.spatial.transform import Rotation as R
from GraspNetToolBox.config import pos_kinect, q_kinect, pos_realsense, q_realsense, ROBOT_START_POINT, ROBOT_START_ROTATION


def q_to_matrix(rot):
    qw = rot[0]
    qx = rot[1]
    qy = rot[2]
    qz = rot[3]
    trans_matrix = list()
    trans_matrix.append([
        1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy + 2 * qw * qz,
        2 * qx * qz - 2 * qw * qy
    ])
    trans_matrix.append([
        2 * qx * qy - 2 * qw * qz, 1 - 2 * qx * qx - 2 * qz * qz,
        2 * qy * qz + 2 * qw * qx
    ])
    trans_matrix.append([
        2 * qx * qz + 2 * qw * qy, 2 * qy * qz - 2 * qw * qx,
        1 - 2 * qx * qx - 2 * qy * qy
    ])
    trans_matrix = np.array(trans_matrix)
    return trans_matrix


def matrix_to_q(rot):
    r = R.from_matrix(rot)
    quat = r.as_quat()
    return np.array([quat[3], quat[0], quat[1], quat[2]])


def get_trans_matrix(pos, rot):
    # get matrix
    trans_matrix = q_to_matrix(rot)
    inv_matrix = np.linalg.inv(trans_matrix)
    # get offset
    trans_offset = pos
    return trans_matrix, inv_matrix, trans_offset


# get matrix and offset
trans_matrix_kinect, inv_matrix_kinect, trans_offset_kinect = get_trans_matrix(
    pos_kinect, q_kinect)
trans_matrix_realsense, inv_matrix_realsense, trans_offset_realsense = get_trans_matrix(
    pos_realsense, q_realsense)
trans_matrix_hand, inv_matrix_hand, trans_offset_hand = get_trans_matrix(
    ROBOT_START_POINT, ROBOT_START_ROTATION)
# gripper center offset for realsense
trans_offset_realsense += np.array([0, 0, 0.19])


def qmul(q0, q1):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([
        -x0 * x1 - y0 * y1 - z0 * z1 + w0 * w1,
        x0 * w1 + y0 * z1 - z0 * y1 + w0 * x1,
        -x0 * z1 + y0 * w1 + z0 * x1 + w0 * y1,
        x0 * y1 - y0 * x1 + z0 * w1 + w0 * z1
    ])


def rot_camera_to_q_base(rot_in_camera):
    """transform camera rot to base rot.

    Args:
        rot_in_camera (np.array): 3*3 rot_matrix

    Returns:
        q_in_base (np.array): w,x,y,z quat
    """
    q1 = q_kinect
    q2 = matrix_to_q(rot_in_camera)
    return qmul(q1, q2)


def ord_camera_to_base(source, ord_in_camera):
    """transform camera ord to base ord.

    Args:
        source (str): kinect/realsense
        ord_in_camera (np.array): x,y,z

    Returns:
        ord_in_base (np.array): x,y,z
    """
    if len(ord_in_camera) != 3:
        raise ValueError('Input should be a np.array with len == 3')
    if source == 'kinect':
        return np.matmul(inv_matrix_kinect,
                         ord_in_camera) + trans_offset_kinect
    return ord_hand_to_base(ord_camera_to_hand(ord_in_camera))


def ord_camera_to_hand(ord_in_camera):
    """transform camera ord to hand ord.

    Args:
        ord_in_camera (np.array): x,y,z

    Returns:
        ord_in_hand (np.array): x,y,z
    """
    if len(ord_in_camera) != 3:
        raise ValueError('Input should be a np.array with len == 3')
    return np.matmul(inv_matrix_realsense,
                     ord_in_camera) + trans_offset_realsense


def ord_hand_to_base(ord_in_hand):
    """transform hand ord to base ord.

    Args:
        ord_in_hand (np.array): x,y,z

    Returns:
        ord_in_base (np.array): x,y,z
    """
    if len(ord_in_hand) != 3:
        raise ValueError('Input should be a np.array with len == 3')
    return np.matmul(inv_matrix_hand, ord_in_hand) + trans_offset_hand


if __name__ == '__main__':
    print(inv_matrix_kinect)
    matrix_in_camera = np.array([[0, 0, 1], [0, 1, 0], [1, 0, 0]])
    # print(np.matmul(matrix_in_camera, matrix_in_camera.T))
    print('q in camera:')
    print(matrix_to_q(matrix_in_camera))
    q_in_base = rot_camera_to_q_base(matrix_in_camera)
    print('q in base:')
    print(q_in_base)
