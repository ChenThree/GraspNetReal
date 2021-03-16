import logging
import time
from RTIF.API import API

# this script demonstrate the usage of 'get' functions
IP = "101.6.69.13"

def main(ROBOT_HOST = IP):

    keep_running = True

    logging.getLogger().setLevel(logging.INFO)
    api = API(ROBOT_HOST)

    scripy_path = "./hammer.script"
    print ("Now start running script")
    api.run_script(scripy_path)
    # sleep for 5 min to keep script running
    if keep_running:
        time.sleep(300)


if __name__ == "__main__":
    main()
