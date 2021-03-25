import numpy as np

# get q and x,y,z from config
from GraspNetToolBox.config import pos_kinect, rot_kinect, pos_realsense, rot_realsense, ROBOT_START_POINT, ROBOT_START_ROTATION


def get_trans_matrix(pos, rot):
    # get trans matrix
    x = pos[0]
    y = pos[1]
    z = pos[2]
    qw = rot[0]
    qx = rot[1]
    qy = rot[2]
    qz = rot[3]
    # get matrix
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
    # 求逆
    inv_matrix = np.linalg.inv(trans_matrix)
    trans_offset = np.array([x, y, z])
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
        return np.matmul(inv_matrix_kinect, ord_in_camera) + trans_offset_kinect
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
    print(ord_camera_to_base('kinect', np.array([0, 0, 0.85])))
