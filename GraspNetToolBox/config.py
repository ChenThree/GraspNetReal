# robot ip
IP_ADDRESS = '101.6.70.97'
# robot initial status
ROBOT_START_POINT = [-0.07566725, -0.2933787, 0.5278667]
ROBOT_START_ROTATION = [0.1582771, 0.98715806, 0.02128513, 0.0037758]

# gripper port
GRIPPER_PORT = '/dev/ttyUSB0'

# kinect ord transform matrix
x, y, z = [0.7176088821850692, -0.46630958181012533, 0.5907089176324598]
qw, qx, qy, qz = [
    0.27388242765243626, -0.6475533833289985, -0.6581554175646015,
    0.2692479858657072
]

# kinect raw calibration
cx = 0.49866631627082825  # 1 (order in calibration json paras)
cy = 0.50889438390731812  # 2
fx = 0.47901439666748047  # 3
fy = 0.63846969604492188  # 4
# picture resolution
resolution_x = 1280
resolution_y = 720

# grasp mask
KINECT_MASK_IAMGE_PATH = './GraspNetToolBox/utils/kinect_mask.png'
REALSENSE_MASK_IAMGE_PATH = './GraspNetToolBox/utils/realsense_mask.png'
