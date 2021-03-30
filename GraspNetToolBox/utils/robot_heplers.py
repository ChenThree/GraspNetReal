import time

# get robot ip address
from GraspNetToolBox.config import IP_ADDRESS, ROBOT_START_POINT, ROBOT_START_ROTATION
# lower api
from GraspNetToolBox.RTIF.HAPI import HAPI


class RobotController():

    def __init__(self):
        self.controller = HAPI(IP_ADDRESS)

    def get_pos(self):
        return self.controller.GetCurrentEndPos()[0]

    def get_rot(self):
        return self.controller.GetCurrentEndPos()[1]

    def wait_for_movement(self):
        # wait for movement to complete
        while not self.controller.isLastMovementEnd():
            time.sleep(0.5)
        time.sleep(1)

    def reset_robot(self, a=1.2, v=0.05, t=None):
        # move robot
        print('move to start point')
        self.controller.MoveEndPointToPosition(
            pos=ROBOT_START_POINT,
            rotation=ROBOT_START_ROTATION,
            a=a,
            v=v,
            t=t)
        self.wait_for_movement()
        print('move to start point completed')

    def move_robot(self, pos=None, rotation=None, a=1.2, v=0.05, t=None):
        print('move to given point')
        self.wait_for_movement()
        self.controller.MoveEndPointToPosition(
            rotation=rotation, pos=pos, a=a, v=v, t=t)
        self.wait_for_movement()
        print('move to given point completed')


if __name__ == '__main__':
    from GraspNetToolBox.utils.ord_helpers import q_to_euler, euler_to_q
    import numpy as np
    controller = RobotController()
    pos = controller.get_pos()
    q = controller.get_rot()
    print('now_pos ==', pos)
    print('now_rot ==', q)
    euler = q_to_euler(q)
    print(euler)
    euler = np.array([-180, 0, 0])
    q = euler_to_q(euler)
    print(q)
    controller.reset_robot()
    # base
    # controller.move_robot(rotation=q)
