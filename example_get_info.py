import logging
import time
from RTIF.HAPI import HAPI

# this script demonstrate the usage of 'get' functions
IP = "101.6.69.245"

def main(ROBOT_HOST = IP):

    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = HAPI(ROBOT_HOST)

    while True:
        time.sleep(0.5)
        ep = api.GetCurrentEndPos()
        ef = api.GetCurrentEndForce()
        print ("Current Tool Position=(x=%f, y=%f, z=%f), (w=%f, i=%f, j=%f, k=%f)" %
               (ep[0][0], ep[0][1], ep[0][2], ep[1][0], ep[1][1], ep[1][2], ep[1][3]))
        print ("Current Tool Force=(x=%f, y=%f, z=%f), (rx=%f, ry=%f, rz=%f)" %
               (ef[0], ef[1], ef[2], ef[3], ef[4], ef[5]))
        print ("")

if __name__ == "__main__":
    main()
