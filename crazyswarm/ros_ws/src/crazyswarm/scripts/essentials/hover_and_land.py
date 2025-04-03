from pycrazyswarm import *

LAND_SPEED = 0.5 #m/s
Z_SPEED = 0.75 # m/s
HEIGHT = 0.7 #m
LAND_HEIGHT = 0.04 #m


# Take off and hover. If y is pressed, all drones land
def hover_all(swarm):
    timeHelper = swarm.timeHelper
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=HEIGHT, duration=HEIGHT / Z_SPEED)
        timeHelper.sleep(HEIGHT / Z_SPEED)
    user_input = input("Continue? [y/n]")
    if (user_input != "y"):
        print("Performing landing...")
        land_all(swarm)
        exit()


# Land all the drones
def land_all(swarm):
    timeHelper = swarm.timeHelper
    max_duration = 0.0
    for cf in swarm.allcfs.crazyflies:
        z = cf.position()[2]
        duration = z / Z_SPEED + 1
        max_duration = max(max_duration, duration)
        cf.land(targetHeight=LAND_HEIGHT, duration=duration)
    timeHelper.sleep(max_duration)