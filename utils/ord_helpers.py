import numpy as np

# 标定得到坐标
x, y, z = [0.7176088821850692, -0.46630958181012533, 0.5907089176324598]
# 标定得到四元数
qw, qx, qy, qz = [
    0.27388242765243626, -0.6475533833289985, -0.6581554175646015,
    0.2692479858657072
]
# 得到变换矩阵
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

def camera_to_base(ord_in_camera):
    """transform camera ord to base ord

    Args:
        ord_in_camera (np.array): x,y,z

    Returns:
        ord_in_base (np.array): x,y,z
    """
    if len(ord_in_camera) != 3:
        raise ValueError('Input should be a np.array with len == 3')
    return np.matmul(inv_matrix, ord_in_camera) + trans_offset 


def base_to_camera(ord_in_base):
    """transform base ord to camera ord

    Args:
        ord_in_base (np.array): x,y,z

    Returns:
        ord_in_camera (np.array): x,y,z
    """
    if len(ord_in_base) != 3:
        raise ValueError('Input should be a np.array with len == 3')
    return np.matmul(trans_matrix, ord_in_base - trans_offset) 

if __name__ == '__main__':
    print('-'*80)
    print('matrix:')
    print(trans_matrix)
    print('inv_matrix:')
    print(inv_matrix)
    print('offset:')
    print(trans_offset)
    print('-'*80)
    print(camera_to_base(np.array([0, 0, 0])))
    print(camera_to_base(np.array([0, 0, 0.83])))
    print(base_to_camera(np.array([0.71760888, -0.46630958, 0.59070892])))
    print(base_to_camera(np.array([0.2, -0.2, 0])))