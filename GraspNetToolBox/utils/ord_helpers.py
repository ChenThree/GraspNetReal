import numpy as np

# get q and x,y,z from config
from GraspNetToolBox.config import qw, qx, qy, qz, x, y, z

# get trans matrix
trans_matrix = np.array([[
    1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy + 2 * qw * qz,
    2 * qx * qz - 2 * qw * qy
],
                         [
                             2 * qx * qy - 2 * qw * qz,
                             1 - 2 * qx * qx - 2 * qz * qz,
                             2 * qy * qz + 2 * qw * qx
                         ],
                         [
                             2 * qx * qz + 2 * qw * qy,
                             2 * qy * qz - 2 * qw * qx,
                             1 - 2 * qx * qx - 2 * qy * qy
                         ]])
# 求逆
inv_matrix = np.linalg.inv(trans_matrix)
trans_offset = np.array([x, y, z])

def rot_camera_to_base(rot_in_camera):
    """transform camera rot to base rot.

    Args:
        rot_in_camera (np.array): x,y,z

    Returns:
        rot_in_base (np.array): x,y,z
    """
    return np.matmul(inv_matrix, rot_in_camera)


def rot_base_to_camera(rot_in_base):
    """transform base rot to camera rot.

    Args:
        rot_in_base (np.array): x,y,z

    Returns:
        rot_in_camera (np.array): x,y,z
    """
    return np.matmul(trans_matrix, rot_in_base)


def ord_camera_to_base(ord_in_camera):
    """transform camera ord to base ord.

    Args:
        ord_in_camera (np.array): x,y,z

    Returns:
        ord_in_base (np.array): x,y,z
    """
    if len(ord_in_camera) != 3:
        raise ValueError('Input should be a np.array with len == 3')
    return np.matmul(inv_matrix, ord_in_camera) + trans_offset


def ord_base_to_camera(ord_in_base):
    """transform base ord to camera ord.

    Args:
        ord_in_base (np.array): x,y,z

    Returns:
        ord_in_camera (np.array): x,y,z
    """
    if len(ord_in_base) != 3:
        raise ValueError('Input should be a np.array with len == 3')
    return np.matmul(trans_matrix, ord_in_base - trans_offset)


if __name__ == '__main__':
    print('-' * 80)
    print('matrix:')
    print(trans_matrix)
    print('inv_matrix:')
    print(inv_matrix)
    print('offset:')
    print(trans_offset)
    print('-' * 80)
    print(ord_camera_to_base(np.array([0, 0, 0])))
    print(ord_camera_to_base(np.array([0, 0, 0.83])))
    print(ord_base_to_camera(np.array([0.71760888, -0.46630958, 0.59070892])))
    print(ord_base_to_camera(np.array([0.2, -0.2, 0])))
