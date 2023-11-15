import numpy as np
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
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

def interpolate(start, goal, steps=100):
    """ Generate intermediate configurations between start and goal """
    x_values = np.linspace(start[0], goal[0], steps)
    y_values = np.linspace(start[1], goal[1], steps)
    theta_values = np.linspace(start[2], goal[2], steps)
    return np.vstack((x_values, y_values, theta_values)).T

def update(frame, robot, configurations, ax):
    """ Update function for animation """
    ax.clear()
    ax.set_xlim(0, 2)
    ax.set_ylim(0, 2)
    ax.set_aspect('equal', adjustable='box')
    config = configurations[frame]
    robot.set_position(*config)
    robot.draw(ax)

def main():
    parser = argparse.ArgumentParser(description='Animate robot moving from start to goal.')
    parser.add_argument('--start', nargs=3, type=float, help='Start configuration [x, y, theta]')
    parser.add_argument('--goal', nargs=3, type=float, help='Goal configuration [x, y, theta]')
    args = parser.parse_args()

    start = np.array(args.start)
    goal = np.array(args.goal)

    robot = RigidBodyRobot()
    configurations = interpolate(start, goal)

    fig, ax = plt.subplots()
    ani = animation.FuncAnimation(fig, update, frames=len(configurations), fargs=(robot, configurations, ax), interval=50)
    
    plt.show()

if __name__ == "__main__":
    main()
