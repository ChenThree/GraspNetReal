# robot ip
IP_ADDRESS = '101.6.68.222'
# robot initial status
ROBOT_START_POINT = [-0.05136441, -0.4219078, 0.35249886]
ROBOT_START_ROTATION = [0.00389084, -0.06481626, 0.9939224, -0.08889267]

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
MASK_IAMGE_PATH = './GraspNetToolBox/utils/mask.png'
