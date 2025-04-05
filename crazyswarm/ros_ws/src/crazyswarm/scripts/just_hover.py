from pycrazyswarm import *
from hover_and_land import *
import numpy as np
from emergency_break import *
from std_msgs.msg import Bool

# Tesing the hover and emergency break 
def main():
    swarm = Crazyswarm()
    print("Testing hovering...")
    timeHelper = swarm.timeHelper

    # Until the status is turned False, the drones keep moving to -x direction
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=HEIGHT, duration=HEIGHT / Z_SPEED)
        timeHelper.sleep(HEIGHT / Z_SPEED)
    while True:
        swarm.allcfs.goTo(np.array([-0.1,0.0,0.0]),0.0, 0.4)
        timeHelper.sleep(0.4)
        if not swarm.status:
            break
    land_all(swarm)

if __name__ == "__main__":
    main()
