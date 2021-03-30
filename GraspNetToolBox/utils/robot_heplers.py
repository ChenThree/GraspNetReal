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
    controller = RobotController()
    print('now_pos ==', controller.get_pos())
    print('now_rot ==', controller.get_rot())
    # controller.reset_robot()
    # camera
    # controller.move_robot(rotation=[-0.58028898, 0.23624997, -0.28031522, 0.68852426])
    # base
    controller.move_robot(
        rotation=[0.65815542, 0.26924799, -0.27388243, 0.64755338])
