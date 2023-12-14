import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def parse_args():
    parser = argparse.ArgumentParser(description='Evaluate Particle Filter Localization')
    parser.add_argument('--map', type=str, required=True, help='Path to the landmark map file')
    parser.add_argument('--execution', type=str, required=True, help='Path to the ground truth poses file')
    parser.add_argument('--estimates', type=str, required=True, help='Path to the particle filter estimates file')
    return parser.parse_args()

def load_data(map_file, execution_file, estimates_file):
    map_landmarks = np.load(map_file)
    ground_truth_poses = np.load(execution_file)
    estimated_poses = np.load(estimates_file)
    return map_landmarks, ground_truth_poses, estimated_poses

def create_animation(map_landmarks, ground_truth_poses, estimated_poses):
    fig, ax = plt.subplots()
    def animate(i):
        ax.clear()
        ax.scatter(map_landmarks[:, 0], map_landmarks[:, 1], marker='o', color='blue')
        ax.plot(ground_truth_poses[:i, 0], ground_truth_poses[:i, 1], color='blue', label='Ground Truth')
        ax.plot(estimated_poses[:i, 0], estimated_poses[:i, 1], color='black', label='Estimate')
        ax.legend()
    anim = FuncAnimation(fig, animate, frames=len(ground_truth_poses), interval=100)
    return anim

def calculate_errors(ground_truth_poses, estimated_poses):
    translational_errors = np.linalg.norm(ground_truth_poses[:, :2] - estimated_poses[:, :2], axis=1)
    rotational_errors = np.abs(ground_truth_poses[:, 2] - estimated_poses[:, 2])
    return translational_errors, rotational_errors

def plot_errors(translational_errors, rotational_errors):
    fig, (ax1, ax2) = plt.subplots(2, 1)
    ax1.plot(translational_errors, label='Translational Error')
    ax1.set_ylabel('Distance')
    ax1.set_title('Translational Error over Time')
    ax1.legend()

    ax2.plot(rotational_errors, label='Rotational Error')
    ax2.set_xlabel('Time Step')
    ax2.set_ylabel('Radians')
    ax2.set_title('Rotational Error over Time')
    ax2.legend()

    plt.tight_layout()
    plt.show()

def main():
    args = parse_args()
    map_landmarks, ground_truth_poses, estimated_poses = load_data(args.map, args.execution, args.estimates)
    
    anim = create_animation(map_landmarks, ground_truth_poses, estimated_poses)
    translational_errors, rotational_errors = calculate_errors(ground_truth_poses, estimated_poses)
    plot_errors(translational_errors, rotational_errors)


    anim.save('particle_filter_animation.mp4')

if __name__ == "__main__":
    main()
