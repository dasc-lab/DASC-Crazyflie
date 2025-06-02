from pycrazyswarm import Crazyswarm
import numpy as np
import matplotlib.pyplot as plt
import hungarian

N_CFS = 9  # Number of Crazyflies in the swarm

L_SHAPE = np.array([
    [1.0, 1.0],
    [0.6, 1.0],
    [0.2, 1.0],
    [-0.2, 1.0],
    [-0.6, 1.0],
    [-1.0, 1.0],
    [-1.0, 0.6],
    [-1.0, 0.2],
    [-1.0, -0.2]
])

FOUR_SHAPE = np.array([
    [1.0, 0.0],
    [0.5, 0.0],
    [0.0, 0.0],
    [-0.5, 0.0],
    [-1.0, 0.0],
    [0.0, -0.5],
    [0.0, 0.5],
    [0.0, 1.0],
    [0.5, 0.5]
])

D_SHAPE = np.array([
    [1.0, 1.0],
    [0.33, 1.0],
    [-0.33, 1.0],
    [-1.0, 1.0],
    [-0.8, 0.5],
    [-0.5, 0.0],
    [0.0, -0.3],
    [0.8, 0.5],
    [0.5, 0.0]
])

C_SHAPE = np.array([
    [0.866, -0.5],
    [1.0, 0.0],
    [0.866, 0.5],
    [0.5, 0.866],
    [0.0, 1.0],
    [-0.5, 0.866],
    [-0.866, 0.5],
    [-1.0, 0.0],
    [-0.866, -0.5]
])

def compute_optimal_assignment(swarm, goal_positions):
    """
    Computes the optimal assignment of Crazyflies to target positions using the Hungarian algorithm.
    :param swarm: Crazyswarm instance containing all Crazyflies.
    :param positions: Target positions for the Crazyflies.
    :return: List of indices representing the optimal assignment.
    """
    current_positions = get_current_positions(swarm)  # Get the current positions of the Crazyflies
    C = np.zeros((N_CFS, N_CFS))  # Cost matrix for the assignment problem
    for i in range(len(current_positions)):
        for j in range(N_CFS):
            C[i,j] = np.hypot(current_positions[i][0] - goal_positions[j][0],
                              current_positions[i][1] - goal_positions[j][1])

    H = hungarian.Hungarian(C)  # Calculate the optimal assignment using the Hungarian algorithm
    H.calculate()  # Perform the assignment calculation
    return H.get_results()

def get_current_positions(swarm):
    """
    Retrieves the current positions of all Crazyflies in the swarm.
    :param swarm: Crazyswarm instance containing all Crazyflies.
    :return: List of current positions of the Crazyflies.
    """
    return np.array([cf.position() for cf in swarm.allcfs.crazyflies])

def transform_shape(shape, pitch, roll, yaw, offset = np.array([0.0,0.0,1.0])):
    """
    Transforms a given shape by applying a translation and rotation.
    :param shape: The original shape as a numpy array of points.
    :param offset: Translation vector to apply to the shape.
    :param pitch: Pitch angle in radians for rotation.
    :param roll: Roll angle in radians for rotation.
    :param yaw: Yaw angle in radians for rotation.
    :return: Transformed shape as a numpy array of points.
    """
    rotation_matrix = np.array([
        [np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), -np.sin(pitch)],
        [-np.sin(yaw) * np.cos(roll) + np.cos(yaw) * np.sin(pitch) * np.sin(roll),
         np.cos(yaw) * np.cos(roll) + np.sin(yaw) * np.sin(pitch) * np.sin(roll),
         np.cos(pitch) * np.sin(roll)],
        [np.sin(yaw) * np.sin(roll) + np.cos(yaw) * np.sin(pitch) * np.cos(roll),
         -np.cos(yaw) * np.sin(roll) + np.sin(yaw) * np.sin(pitch) * np.cos(roll),
         np.cos(pitch) * np.cos(roll)]
    ])
    
    return (shape @ rotation_matrix.T + offset).astype(np.float32)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper

    for cf in swarm.allcfs.crazyflies:
        cf.enableCollisionAvoidance(enable=True)

    # Take off
    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=1.0, duration=1.0)
    timeHelper.sleep(1.0)

    # Get current positions

    optimal_assignment = compute_optimal_assignment(swarm, L_SHAPE)     

    for i in range(N_CFS):
        swarm.allcfs.crazyflies[i].goTo(L_SHAPE[i].tolist() + [1.0], 0, 2.0)
    timeHelper.sleep(2.0)

    # Move to the next shape
    for i in range(N_CFS):
        swarm.allcfs.crazyflies[i].goTo(FOUR_SHAPE[i].tolist() + [1.0], 0, 2.0)
    timeHelper.sleep(2.0)

    # Move to the next shape
    for i in range(N_CFS):
        swarm.allcfs.crazyflies[i].goTo(D_SHAPE[i].tolist() + [1.0], 0, 2.0)
    timeHelper.sleep(2.0)

    # Move to the next shape
    for i in range(N_CFS):
        swarm.allcfs.crazyflies[i].goTo(C_SHAPE[i].tolist() + [1.0], 0, 2.0)
    timeHelper.sleep(2.0)

    # Land all
    for cf in swarm.allcfs.crazyflies:
        cf.land(targetHeight=0.04, duration=1.5)
    timeHelper.sleep(1.5)

if __name__ == "__main__":
    main()


