from pycrazyswarm import Crazyswarm
import numpy as np

height = 0.7
second_height = 7.5
LAND_SPEED = 0.5 #m/s
HOVER_DURATION = 2.0
Z_SPEED = 0.75 # m/s

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=height, duration=height / Z_SPEED)
    timeHelper.sleep(1)

    swarm.allcfs.goTo([-1,0,0], 0, duration=2)
    timeHelper.sleep(3)
    # swarm.land_all()

    
if __name__ == "__main__":
    main()


