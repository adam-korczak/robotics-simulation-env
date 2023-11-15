import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
import argparse
import matplotlib.patches as patches
import random


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

def point_in_obstacle(point, obstacle):
    x, y = point
    return obstacle.contains_point((x, y))

def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def intersect(line1_start, line1_end, line2_start, line2_end):
    return ccw(line1_start, line2_start, line2_end) != ccw(line1_end, line2_start, line2_end) and ccw(line1_start, line1_end, line2_start) != ccw(line1_start, line1_end, line2_end)

def intersects_obstacle(line_start, line_end, obstacle):
    for i in range(len(obstacle) - 1):
        if intersect(line_start, line_end, obstacle[i], obstacle[i + 1]):
            return True
    return False


class RRT:
    def __init__(self, start, goal, obstacles, max_iterations=1000, goal_sample_rate=0.05):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.obstacles = obstacles
        self.max_iterations = max_iterations
        self.goal_sample_rate = goal_sample_rate
        self.nodes = [self.start]
        self.parent = {self.start: None}
        self.q_near = self.start

    def extend(self, q_rand):
        step_size = 0.1  # Adjust this value as needed
        delta = np.array(q_rand) - np.array(self.q_near)
        distance = np.linalg.norm(delta)
        if distance < step_size:
            q_new = q_rand  # q_rand is very close to q_near
        else:
            direction = delta / distance
            q_new = np.array(self.q_near) + direction * step_size

        if self.is_collision_free(self.q_near, q_new):
            return q_new  # Successfully extended the tree
        else:
            return None  # Edge is not collision-free
        

    def is_collision_free(self, q1, q2):
        # Check if the edge between q1 and q2 is collision-free
        for obstacle in self.obstacles:
            if intersects_obstacle(q1, q2, obstacle):
                return False
        return True

    def search(self):
        for _ in range(self.max_iterations):
            q_rand = self.sample_random_configuration()
            q_near = self.find_nearest_neighbor(q_rand)
            q_new = self.extend(q_rand)
            if q_new is not None and self.is_collision_free(q_near, q_new):
                self.nodes.append(q_new)
                self.parent[tuple(q_new)] = tuple(q_near)
                if self.is_near_goal(q_new):  # This assumes q_new is the goal configuration
                    return



    def find_path(self):
        # Find a path from start to goal
        if self.goal not in self.parent:
            return None  # Goal is not reached
        path = [self.goal]
        current = self.goal
        while current != self.start:
            current = self.parent[current]
            path.insert(0, current)
        return path

    def sample_random_configuration(self):
        # Sample a random configuration
        if random.random() < self.goal_sample_rate:
            return self.goal
        return [random.uniform(0, 2 * np.pi), random.uniform(0, 2 * np.pi)]

    def find_nearest_neighbor(self, q_rand):
        nearest_node = None
        min_distance = float('inf')

        for node in self.nodes:
            distance = np.linalg.norm(np.array(q_rand) - np.array(node))
            if distance < min_distance:
                min_distance = distance
                nearest_node = node

        return nearest_node

    def is_near_goal(self, q_new):
        proximity_threshold = 0.1  # Adjust this value as needed

        distance_to_goal = np.linalg.norm(np.array(q_new) - np.array(self.goal))

        return distance_to_goal <= proximity_threshold

    def visualize_tree_growth(self):
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim([0, 2])
        ax.set_ylim([0, 2])

        for node in self.nodes:
            if tuple(node) in self.parent and self.parent[tuple(node)] is not None:
                parent = self.parent[tuple(node)]
                plt.plot([parent[0], node[0]], [parent[1], node[1]], 'k-', linewidth=1)

        for obstacle in self.obstacles:
            obstacle_polygon = plt.Polygon(obstacle, closed=True, fill=None, edgecolor='red')
            ax.add_patch(obstacle_polygon)

        plt.plot(self.start[0], self.start[1], 'go', markersize=8)
        plt.plot(self.goal[0], self.goal[1], 'ro', markersize=8)

        plt.title('RRT Tree Growth')
        plt.grid()
        plt.show()

    def visualize_solution_path(self, path):
        if path is None:
            print("No valid path found. Unable to visualize.")
            return
    
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim([0, 2])
        ax.set_ylim([0, 2])

        for obstacle in self.obstacles:
            # Plot the obstacles
            obstacle_polygon = plt.Polygon(obstacle, closed=True, fill=None, edgecolor='red')
            ax.add_patch(obstacle_polygon)

        link_arm = LinkArm(link_lengths=[1, 1], joint_angles=[0, 0], joint_radius=0.1, link_width=0.1, color='blue', ax=ax, fig=fig)
        plt.plot(self.start[0], self.start[1], 'go', markersize=8)
        plt.plot(self.goal[0], self.goal[1], 'ro', markersize=8)

        def update(frame):
            nonlocal link_arm
            link_arm.update_joints(path[frame])
            return link_arm.plot(),

        ani = animation.FuncAnimation(fig, update, frames=len(path), interval=200, repeat=False)

        plt.title('RRT Solution Path')
        plt.grid()
        plt.show()

parser = argparse.ArgumentParser(description="RRT for robot arm motion planning with obstacles.")
parser.add_argument("--start", nargs=2, type=float, help="Start configuration (joint angles)")
parser.add_argument("--goal", nargs=2, type=float, help="Goal configuration (joint angles)")
parser.add_argument("--map", type=str, help="File containing map data (e.g., 'arm_polygons.npy')")
args = parser.parse_args()

start_configuration = args.start
goal_configuration = args.goal
map_filename = args.map

# Load obstacles from the map file
obstacles = np.load(map_filename, allow_pickle=True)

# Create RRT instance and perform the search
rrt = RRT(start_configuration, goal_configuration, obstacles)
rrt.search()
path = rrt.find_path()

# Visualize tree growth and solution path
rrt.visualize_tree_growth()
rrt.visualize_solution_path(path)
