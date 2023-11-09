import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation
import argparse
import matplotlib.patches as patches


class LinkArm(object):
    def __init__(self, link_lengths, joint_angles, joint_radius, link_width, color, ax, fig):
        self.link_lengths = np.array(link_lengths)
        self.joint_angles = np.array(joint_angles)
        self.joint_radius = joint_radius
        self.link_width = link_width
        self.n_links = 2
        self.points = np.ones((self.n_links + 1, 2))
        self.color = color
        self.ax = ax
        self.fig = fig

        self.terminate = False

        self.update_points()

    def update_joints(self, joint_angles):
        self.joint_angles = joint_angles
        self.update_points()
    def transformation_matrix(self, theta, length):
        return np.array([
            [np.cos(theta), -np.sin(theta), length * np.cos(theta)],
            [np.sin(theta), np.cos(theta), length * np.sin(theta)],
            [0, 0, 1]
        ])
    
    # transformation matrix approach
    def update_points(self):
        point = np.array([0, 0, 1]).reshape(3, 1)
        prev_trans = np.identity(3) # Initialize as identity matrix
        for i in range(self.n_links):
            trans = self.transformation_matrix(self.joint_angles[i], self.link_lengths[i])
            prev_trans = prev_trans @ trans
            new_point = prev_trans @ point
            new_point[0, 0] += self.points[0][0]
            new_point[1, 0] += self.points[0][1]
            self.points[i + 1][0] = new_point[0, 0]
            self.points[i + 1][1] = new_point[1, 0]
    
    def draw_rectangle(self, start, end):
        """Create a rectangle from start to end with a certain width."""
        direction = np.array(end) - np.array(start)
        length = np.linalg.norm(direction)
        direction = direction / length

        # Adjust the start and end points to account for the circle radius
        start_adj = start + self.joint_radius * direction
        end_adj = end - self.joint_radius * direction

        # Calculate the perpendicular direction
        perp_direction = np.array([-direction[1], direction[0]])
        half_width_vec = 0.3 * self.link_width * perp_direction

        # Calculate the 4 corners of the rectangle
        p1 = start_adj - half_width_vec
        p2 = start_adj + half_width_vec
        p3 = end_adj + half_width_vec
        p4 = end_adj - half_width_vec

        return np.array([p1, p4, p3, p2, p1])
 

    def plot(self):

        for i in range(self.n_links):
            rectangle = self.draw_rectangle(self.points[i], self.points[i + 1])
            self.ax.plot(rectangle[:, 0], rectangle[:, 1], self.color)
            self.ax.fill(rectangle[:, 0], rectangle[:, 1], self.color, alpha=0.3)  # Filling the rectangle

        for i in range(self.n_links + 1):
            circle = patches.Circle(self.points[i], radius=self.joint_radius, facecolor=self.color)
            self.ax.add_patch(circle)
        self.ax.set_xlim([0, 2])
        self.ax.set_ylim([0, 2])
        plt.pause(1e-5)



parser = argparse.ArgumentParser(description="Animate robot arm from start to goal configuration.")
parser.add_argument("--start", nargs=2, type=float, help="Start configuration (joint angles)")
parser.add_argument("--goal", nargs=2, type=float, help="Goal configuration (joint angles)")
args = parser.parse_args()

def interpolate(start, goal, steps):
    delta = (goal - start) / steps
    for i in range(steps + 1):
        yield start + i * delta

fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([0, 2])
ax.set_ylim([0, 2])

arm =  LinkArm([0.4,0.25], args.start, joint_radius=0.05, link_width=0.1, color = 'black', ax=ax, fig=fig)

def update(frame):
    ax.clear()
    arm.update_joints(next(config_interpolator))
    arm.plot()

goal = np.array(args.goal)
start = np.array(args.start)
steps = 50
config_interpolator = interpolate(start, goal, steps)
animation = FuncAnimation(fig, update, frames=steps+1, repeat=False)
plt.show()
