import numpy as np
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle

# Load the list of random configurations
configurations = np.load("arm_configs.npy")

class NLinkArm(object):
    def __init__(self, link_lengths, joint_angles, joint_radius, link_width, color, ax, fig):
        self.n_links = 2
        if self.n_links != len(joint_angles):
            raise ValueError()

        self.link_lengths = np.array(link_lengths)
        self.joint_angles = np.array(joint_angles)
        self.joint_radius = joint_radius
        self.link_width = link_width
        self.points = np.ones((self.n_links + 1, 2))
        self.color = color
        self.ax = ax
        self.fig = fig

        self.terminate = False
        #self.fig, self.ax = plt.subplots()
        #self.ax.set_aspect('equal')
        #self.ax.set_xlim([0, 2])
        #self.ax.set_ylim([0, 2])
        #self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.update_points()
        #self.plot()
        

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
 
    def on_key(self, event):
        if event.key == 'q':
            self.terminate = True
            plt.close()
            return
        elif event.key == 'z':
            self.rotate_joint(0, -1)
        elif event.key == 'x':
            self.rotate_joint(0, 1)
        elif event.key == 'c':
            self.rotate_joint(1, -1)
        elif event.key == 'v':
            self.rotate_joint(1, 1)
        elif event.key == 'b':
            self.rotate_joint(2, -1)
        elif event.key == 'n':
            self.rotate_joint(2, 1)
        self.plot()


    def run(self):
        while not self.terminate:
            plt.pause(0.1)

    def plot(self):
        #self.ax.clear()

        for i in range(self.n_links):
            rectangle = self.draw_rectangle(self.points[i], self.points[i + 1])
            self.ax.plot(rectangle[:, 0], rectangle[:, 1], self.color)
            self.ax.fill(rectangle[:, 0], rectangle[:, 1], self.color, alpha=0.3)  # Filling the rectangle

        for i in range(self.n_links + 1):
            circle = patches.Circle(self.points[i], radius=self.joint_radius, facecolor=self.color)
            self.ax.add_patch(circle)
        self.ax.set_xlim([0, 2])
        self.ax.set_ylim([0, 2])
        """
        for polygon in map_polygons:
            plt.fill(*zip(*polygon), "r")
        """
        #plt.draw()
        plt.pause(1e-5)

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Find nearest robot configurations.")
parser.add_argument("--target", nargs=2, type=float, help="Target configuration (joint angles)")
parser.add_argument("--k", type=int, help="Number of nearest neighbors to report")
parser.add_argument("--configs", type=str, help="File containing random configurations")
args = parser.parse_args()

def distance(config1, config2):
    return np.linalg.norm(config1 - config2)

target_config = np.array(args.target)
k = args.k
distances = [distance(target_config, config) for config in configurations]
nearest_indices = np.argsort(distances)[:k]

fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([0, 2])
ax.set_ylim([0, 2])

target_arm = NLinkArm([0.4, 0.25], args.target, joint_radius=0.05, link_width=0.1, color = 'black', ax=ax, fig=fig)
target_arm.plot()

colors = ['red', 'green', 'blue', 'yellow']

for i, index in enumerate(nearest_indices):
    if i >= len(colors):
        color = 'yellow'
    else:
        color = colors[i]
    
    neighbor_config = configurations[index]
    neighbor_arm = NLinkArm([0.4, 0.25],[neighbor_config[0], neighbor_config[1]], joint_radius=0.05, link_width=0.1, color=color, ax=ax, fig=fig)
    neighbor_arm.plot()


plt.draw()
plt.show()
