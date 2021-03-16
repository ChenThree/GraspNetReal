import numpy as np

# cal intrinsics_matrix from raw calibration
cx = 0.50506317615509033 # 1
cy = 0.51044702529907227 # 2
fx = 0.49250051379203796 # 3
fy = 0.49251702427864075 # 4
# inv normalize
resolution_x = 1280
resolution_y = 720
cx *= resolution_x
fx *= resolution_x
cy *= resolution_y
fy *= resolution_y

def get_intrinsics_matrix():
    """get intrinsics_matrix according to settings above

    Returns:
        intrinsics_matrix: 3*3 matrix
    """
    intrinsics_matrix = np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    return intrinsics_matrix

if __name__ == '__main__':
    print(get_intrinsics_matrix())