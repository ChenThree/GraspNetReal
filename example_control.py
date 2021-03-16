import logging
import time
from RTIF.API import API

# this script demonstrate how to control the robot arm to a specified location
# by writing the angle of each joint.
#
# Notice: the default initial position is a singular position which we can only
# control the joint angle but cannot set tool location (the Invert Kines solver
# will report error)
IP = "101.6.70.22"

def main(ROBOT_HOST = IP):

    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = API(ROBOT_HOST)

    # control loop
    while keep_running:
        # api.MoveJointToRad((-0.0011, -2.0397, 1.8195, -1.5796, -1.5719, 0.000))
        api.MoveEndPointToPosition(pos=(0.5774472131510267, -0.11612032542570545, 0.31002231707422814),
                                   rotation=(2.2558090087495724, -2.1853634973877805, 0.016568855033798274))
        time.sleep(2)
        api.MoveEndPointToPosition(pos=(0.5774472131510267, -0.11612032542570545, 0.21002231707422814),
                                   rotation=(2.2558090087495724, -2.1853634973877805, 0.016568855033798274))
        time.sleep(2)


if __name__ == "__main__":
    main()
