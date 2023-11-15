import numpy as np
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D

class RigidBodyRobot:
    def __init__(self, length=0.2, width=0.1):
        self.length = length
        self.width = width
        self.x = 0
        self.y = 0
        self.theta = 0

    def set_position(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def get_corners(self):
        corners = np.array([[-self.length / 2, -self.width / 2],
                            [self.length / 2, -self.width / 2],
                            [self.length / 2, self.width / 2],
                            [-self.length / 2, self.width / 2]])
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)],
                      [np.sin(self.theta), np.cos(self.theta)]])
        return np.dot(corners, R) + np.array([self.x, self.y])

    def draw(self, ax):
        rotated_corners = self.get_corners()
        rect = patches.Polygon(rotated_corners, closed=True, color='blue', fill=False)
        front_edge = Line2D([rotated_corners[1, 0], rotated_corners[2, 0]],
                            [rotated_corners[1, 1], rotated_corners[2, 1]],
                            color='red', linewidth=2)
        ax.add_patch(rect)
        ax.add_line(front_edge)

def load_configurations(file_path):
    return np.load(file_path, allow_pickle=True)

def distance_metric(config1, config2, alpha=0.7):
    dt = np.linalg.norm(config1[:2] - config2[:2])  # Euclidean distance for translation
    dr = min(abs(config1[2] - config2[2]), 2 * np.pi - abs(config1[2] - config2[2]))  # Rotational distance
    return alpha * dt + (1 - alpha) * dr

def find_nearest_configs(target, configs, k):
    distances = [distance_metric(target, config) for config in configs]
    nearest_indices = np.argsort(distances)[:k]
    return configs[nearest_indices]

def plot_configurations(configs, target, ax):
    for config in configs:
        robot = RigidBodyRobot()
        robot.set_position(*config)
        robot.draw(ax)
    # Mark the target configuration
    robot = RigidBodyRobot()
    robot.set_position(*target)
    robot.draw(ax)
    ax.scatter(*target[:2], color='red', s=100)  # Mark the target location

def main():
    parser = argparse.ArgumentParser(description='Find and plot nearest robot configurations.')
    parser.add_argument('--target', nargs=3, type=float, help='Target configuration [x, y, theta]')
    parser.add_argument('--k', type=int, help='Number of nearest configurations to find')
    parser.add_argument('--configs', type=str, help='Path to configurations file')
    args = parser.parse_args()

    target = np.array(args.target)
    k = args.k
    configs_file = args.configs

    configs = load_configurations(configs_file)
    nearest_configs = find_nearest_configs(target, configs, k)

    fig, ax = plt.subplots()
    ax.set_xlim(0, 2)
    ax.set_ylim(0, 2)
    ax.set_aspect('equal', adjustable='box')

    plot_configurations(nearest_configs, target, ax)

    plt.show()

if __name__ == "__main__":
    main()
