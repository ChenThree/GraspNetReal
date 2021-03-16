import logging
import time
from RTIF.HAPI import HAPI

# this script demonstrate controlling robot by keypad
#
# Notice: holding a key will make robot accumulate the step length
IP = "101.6.68.228"

def main():
    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = HAPI(IP)
    print(api.GetCurrentEndPos())
    return
    print('init ok')
    api.set_coordinate_origin((0.13261, -0.49141, 0.32612))
    print('ori set ok')
    api.MoveEndPointToPosition(pos=(0.0, 0.0, 0.0),
                               rotation=(2.2144, -2.2144, 0))
    while not api.isLastMovementEnd():
        time.sleep(0.5)
    time.sleep(2)
    api.FineTuningPosition(interactive=True, add_value=(0.0005, 0.0005, 0.0005, 0.001, 0.001, 0.001), factor=10.0)

if __name__ == "__main__":
    main()
