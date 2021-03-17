import time

# get robot ip address
from GraspNetToolBox.config import IP_ADDRESS, ROBOT_START_POINT, ROBOT_START_ROTATION
# lower api
from GraspNetToolBox.RTIF.HAPI import HAPI


class RobotController():

    def __init__(self):
        self.controller = HAPI(IP_ADDRESS)

    def wait_for_movement(self):
        # wait for movement to complete
        while not self.controller.isLastMovementEnd():
            time.sleep(0.5)
        time.sleep(1)

    def reset_robot(self, a=1.2, v=0.1, t=None):
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

    def move_robot(self,
                   pos=ROBOT_START_POINT,
                   rotation=ROBOT_START_ROTATION,
                   a=1.2,
                   v=0.1,
                   t=None):
        print('move to given point')
        # rotate
        self.controller.MoveEndPointToPosition(
            rotation=rotation, a=a, v=v, t=t)
        self.wait_for_movement()
        # move
        self.controller.MoveEndPointToPosition(pos=pos, a=a, v=v, t=t)
        self.wait_for_movement()
        print('move to given point completed')
