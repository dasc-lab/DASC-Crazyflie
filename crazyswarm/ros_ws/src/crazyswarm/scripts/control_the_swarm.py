from pycrazyswarm import Crazyswarm
import numpy as np
import matplotlib.pyplot as plt

height = 0.7 # m 
second_height = 3.0 # m 

Z_SPEED = 0.75 # m/s

def land_all(swarm):
    # land all
    timeHelper = swarm.timeHelper
    max_duration = 0.0
    for cf in swarm.allcfs.crazyflies:
        z = cf.position()[2]
        duration = z / Z_SPEED + 1
        max_duration = max(max_duration, duration)
        cf.land(targetHeight=0.04, duration=duration)
    timeHelper.sleep(max_duration)



def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    # Take off
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=height, duration=height / Z_SPEED)
    timeHelper.sleep(height / Z_SPEED)

    offset = 0.5
    duration = 1.5
    current_pos = np.array([0.0,0.0,height])
    while True:
        user_input = input("Command: forward (w), backward (s), left (a), right (d), up (e), down (q)")
        if user_input == "w":
            swarm.allcfs.goTo([offset,0,0], 0 , duration)
            current_pos +=np.array([offset,0,0])
            print("moving forward... Current Position: ", current_pos)
        elif user_input == "s":
            swarm.allcfs.goTo([-offset,0,0], 0 , duration)
            current_pos +=np.array([-offset,0,0])
            print("moving backward... Current Position: ", current_pos)
        elif user_input == "a":
            swarm.allcfs.goTo([0,offset,0], 0 , duration)
            current_pos +=np.array([0,offset,0])
            print("moving left... Current Position: ", current_pos)
        elif user_input == "d":
            swarm.allcfs.goTo([0,-offset,0], 0 , duration)
            current_pos +=np.array([0,-offset,0])
            print("moving right... Current Position: ", current_pos)
        elif user_input == "e":
            if current_pos[2] +offset > 4:
                print("It is too high - try moving them down. Current Position: ", current_pos)
                continue
            swarm.allcfs.goTo([0,0,offset], 0 , duration)
            current_pos +=np.array([0,0,offset])
            print("moving up... Current Position: ", current_pos)
        elif user_input == "q":
            if current_pos[2]  -offset < 0.5:
                print("It is too low - try moving them up. Current Position: ", current_pos)
                continue
            swarm.allcfs.goTo([0,0,-offset], 0 , duration)
            current_pos +=np.array([0,0,-offset])
            print("moving down... Current Position: ", current_pos)
        elif user_input == "x":
            swarm.allcfs.goTo([-current_pos[0],-current_pos[1],0], 0 , duration+3)
            timeHelper.sleep(duration+3)
            print("Landing the drones...")
            break
        else:
            print("Invalid Input - please try again. Current Position: ", current_pos)
            continue
        timeHelper.sleep(duration+2)
    land_all(swarm)

if __name__ == "__main__":
    main()