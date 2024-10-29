from pycrazyswarm import Crazyswarm
import numpy as np

LAND_SPEED = 0.5 # m/s

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    height = 1
    Z_SPEED =0.5
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=height, duration=height / Z_SPEED)
    timeHelper.sleep(height / Z_SPEED)
    positions = [cf.position() for cf in swarm.allcfs.crazyflies]

    max_duration = 0.0
    for cf in swarm.allcfs.crazyflies:
        z = cf.position()[2]
        duration = 0.5 + z /LAND_SPEED 
        cf.land(targetHeight=0.04, duration=duration)
        max_duration = max(max_duration, duration)
        
    timeHelper.sleep(max_duration)


if __name__ == "__main__":
    main()


