import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation



def dead_reckoning_solution(initial_pose, odometry_measurements):
    poses = np.zeros((len(odometry_measurements)+1, 3))
    poses[0] = initial_pose

    x, y, theta = initial_pose

    for i, (v,omega) in enumerate(odometry_measurements):
        x += v * np.cos(theta)
        y += v * np.sin(theta)
        theta += omega
        
        poses[i+1] = [x,y,theta]

    return poses

def update(frame, ground_truth, dead_reckoning, landmarks, observations):
    plt.clf()
    plt.scatter(ground_truth[frame, 0], ground_truth[frame, 1], color='blue', marker='o', label='Ground Truth')
    plt.scatter(landmarks[:, 0], landmarks[:, 1], color='blue', marker='x', label='Landmark (GT)')

    for i,obs in enumerate(observations[frame]):
        plt.scatter(obs[0], obs[1], color='red', marker='x', label=f'Landmark {i+1} (Noisy)')






def main():
    parser = argparse.ArgumentParser(description='Dead Reckoning Animation')
    parser.add_argument('--map', type=str, help='Path to landmark map file')
    parser.add_argument('--execution', type=str, help='Path to ground truth motion file')
    parser.add_argument('--sensing', type=str, help='Path to readings file')
    args = parser.parse_args()

    landmarks = np.load(args.map)
    ground_truth = np.load(args.execution)
    readings = np.load(args.sensing, allow_pickle=True)

    initial_pose = readings[0]
    odometry_measurements = readings[1::2]
    observation_measurements = readings[2::2]

    dead_reckoning = dead_reckoning_solution(initial_pose, odometry_measurements)
    
    fig, ax = plt.subplots()
    ani = FuncAnimation(fig, update, frames=len(ground_truth), fargs=(ground_truth, dead_reckoning, landmarks, observation_measurements), interval=100, blit=False)

    plt.show()



if __name__ == "__main__":
    main()
