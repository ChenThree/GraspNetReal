import numpy as np
import math

# get q and x,y,z from config
from GraspNetToolBox.config import pos_kinect, rot_kinect, pos_realsense, rot_realsense, ROBOT_START_POINT, ROBOT_START_ROTATION


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
    # select to biggest num to get stable result
    qw = qx = qy = qz = 0
    if rot[0, 0] + rot[1, 1] + rot[2, 2] + 1 > 0:
        qw = math.sqrt(rot[0, 0] + rot[1, 1] + rot[2, 2] + 1)/2
    if rot[0, 0] - rot[1, 1] - rot[2, 2] + 1 > 0:
        qx = math.sqrt(rot[0, 0] - rot[1, 1] - rot[2, 2] + 1)/2
    if -rot[0, 0] + rot[1, 1] - rot[2, 2] + 1 > 0:
        qy = math.sqrt(-rot[0, 0] + rot[1, 1] - rot[2, 2] + 1)/2
    if -rot[0, 0] - rot[1, 1] + rot[2, 2] + 1 > 0:
        qz = math.sqrt(-rot[0, 0] - rot[1, 1] + rot[2, 2] + 1)/2
    max_num = max([qw, qx, qy, qz])
    # print([qw, qx, qy, qz])
    if qw == max_num:
        qx = (rot[1, 2] - rot[2, 1])/(4*qw)
        qy = (rot[2, 0] - rot[0, 2])/(4*qw)
        qz = (rot[0, 1] - rot[1, 0])/(4*qw)
    elif qx == max_num:
        qw = (rot[1, 2] - rot[2, 1])/(4*qx)
        qy = (rot[0, 1] - rot[1, 0])/(4*qx)
        qz = (rot[2, 0] - rot[0, 2])/(4*qx)
    elif qy == max_num:
        qw = (rot[2, 0] - rot[0, 2])/(4*qy)
        qy = (rot[0, 1] - rot[1, 0])/(4*qy)
        qz = (rot[1, 2] - rot[2, 1])/(4*qy)
    else:
        qw = (rot[0, 1] - rot[1, 0])/(4*qz)
        qy = (rot[2, 0] - rot[0, 2])/(4*qz)
        qz = (rot[1, 2] - rot[2, 1])/(4*qz)
    return np.array([qw, qx, qy, qz])


def get_trans_matrix(pos, rot):
    # get matrix
    trans_matrix = q_to_matrix(rot)
    inv_matrix = np.linalg.inv(trans_matrix)
    # get offset
    trans_offset = pos
    return trans_matrix, inv_matrix, trans_offset


# get matrix and offset
trans_matrix_kinect, inv_matrix_kinect, trans_offset_kinect = get_trans_matrix(
    pos_kinect, rot_kinect)
trans_matrix_realsense, inv_matrix_realsense, trans_offset_realsense = get_trans_matrix(
    pos_realsense, rot_realsense)
trans_matrix_hand, inv_matrix_hand, trans_offset_hand = get_trans_matrix(
    ROBOT_START_POINT, ROBOT_START_ROTATION)
# gripper center offset for realsense
trans_offset_realsense += np.array([0, 0, 0.19])


def rot_camera_to_base(rot_in_camera):
    """transform camera rot to base rot.

    Args:
        rot_in_camera (np.array): x,y,z

    Returns:
        rot_in_base (np.array): x,y,z
    """
    return np.matmul(inv_matrix_kinect, rot_in_camera)


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
    rot_in_base = rot_camera_to_base(
            np.array([[-2.7900429e-02, 2.3445182e-02, 9.9933565e-01],
                      [7.6507765e-0, -6.4290589e-01, 3.6443252e-02],
                      [6.4333332e-01, 7.6558614e-01, -3.3464833e-08]]))
    print('matrix:')
    print(rot_in_base)
    q_in_base = matrix_to_q(rot_in_base)
    print('q:')
    print(q_in_base)
    matrix_in_base = q_to_matrix(q_in_base)
    print('matrix:')
    print(matrix_in_base)
    print('q:')
    print(matrix_to_q(matrix_in_base))
