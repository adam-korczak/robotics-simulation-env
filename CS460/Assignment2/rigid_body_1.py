import numpy as np
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

def load_polygons(filename):
    """ Load the polygon data from a .npy file """
    return np.load(filename, allow_pickle=True)

def plot_polygons(polygons, ax):
    """ Plot the polygons on the given axis """
    for polygon in polygons:
        patch = patches.Polygon(polygon, closed=True, color='gray', alpha=0.5)
        ax.add_patch(patch)

def is_collision(robot, polygons):
    """ Check if there is a collision between the robot and any polygon """
    robot_corners = robot.get_corners()
    for polygon in polygons:
        if any([patches.Polygon(polygon).contains_point(corner) for corner in robot_corners]):
            return True
    return False

def place_robot_randomly(robot, polygons, ax):
    """ Place the robot randomly in the workspace without colliding with polygons """
    max_attempts = 1000
    for _ in range(max_attempts):
        x, y = np.random.uniform(0, 2), np.random.uniform(0, 2)
        theta = np.random.uniform(0, 2*np.pi)
        robot.set_position(x, y, theta)
        if not is_collision(robot, polygons):
            robot.draw(ax)
            return
    raise ValueError("Unable to place the robot without collision after several attempts.")

# Load the polygons
polygons = load_polygons('rigid_polygons.npy')

# Create a figure and axis for plotting
fig, ax = plt.subplots()
ax.set_xlim(0, 2)
ax.set_ylim(0, 2)
ax.set_aspect('equal', adjustable='box')

# Plot the polygons
plot_polygons(polygons, ax)

# Create a robot instance
robot = RigidBodyRobot()

# Try to place the robot randomly without collision
place_robot_randomly(robot, polygons, ax)

# Show the plot
plt.show()
