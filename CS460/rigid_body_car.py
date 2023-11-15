import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D

class RigidBodyRobot:
    def __init__(self, length=0.2, width=0.1):
        self.length = length
        self.width = width
        self.x = 1.0  # Initial x position
        self.y = 1.0  # Initial y position
        self.theta = 0  # Initial orientation
        self.fig, self.ax = plt.subplots()

    def set_position(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def draw(self, ax):
        self.ax.clear()
        self.ax.set_xlim(0, 2)
        self.ax.set_ylim(0, 2)
        self.ax.set_aspect('equal', adjustable='box')
        rect = patches.Polygon(rotated_corners, closed=True, color='blue', fill=False)
        front_edge = Line2D([rotated_corners[1, 0], rotated_corners[2, 0]],
                            [rotated_corners[1, 1], rotated_corners[2, 1]],
                            color='red', linewidth=2)
        ax.add_patch(rect)
        ax.add_line(front_edge)

        # Calculate and transform the corners of the rectangle
        corners = np.array([[-self.length / 2, -self.width / 2],
                            [self.length / 2, -self.width / 2],
                            [self.length / 2, self.width / 2],
                            [-self.length / 2, self.width / 2],
                            [-self.length / 2, -self.width / 2]])  # Close the loop
        R = np.array([[np.cos(self.theta), -np.sin(self.theta)],
                      [np.sin(self.theta), np.cos(self.theta)]])
        rotated_corners = np.dot(corners, R) + np.array([self.x, self.y])

        # Create and add the patch
        rect = patches.Polygon(rotated_corners[:-1], closed=True, color='blue', fill=False)
        self.ax.add_patch(rect)

        # Create and add the red front edge line
        front_edge = Line2D([rotated_corners[1, 0], rotated_corners[2, 0]],
                            [rotated_corners[1, 1], rotated_corners[2, 1]],
                            color='red', linewidth=2)
        self.ax.add_line(front_edge)

        plt.draw()

    def on_key(self, event):
        if event.key == 'z':
            self.theta += np.pi / 18  # Rotate counter-clockwise
        elif event.key == 'x':
            self.theta -= np.pi / 18  # Rotate clockwise
        elif event.key == 'c':
            # Move forward in the direction of theta
            self.x += np.cos(self.theta) * 0.1
            self.y += np.sin(self.theta) * 0.1
        elif event.key == 'v':
            # Move backward in the direction opposite to theta
            self.x -= np.cos(self.theta) * 0.1
            self.y -= np.sin(self.theta) * 0.1

        self.draw()

# Create an instance of the robot and draw it
robot = RigidBodyRobot()
robot.draw()

# Connect the key press event
plt.connect('key_press_event', robot.on_key)

# Show the plot
plt.show()
