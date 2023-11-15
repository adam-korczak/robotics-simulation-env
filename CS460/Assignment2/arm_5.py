import numpy as np
import argparse
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import random

#Link arm class:
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





#PRM implementation
class PRM:
    def __init__(self, start, goal, obstacles, max_iterations=1000, k=3):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.obstacles = obstacles
        self.k = k
        self.max_iterations = max_iterations
        self.nodes = [self.start]
        self.edges = {}
        self.q_near = self.start

    def generate_roadmap(self):
        for _ in range(self.max_iterations):
            q_rand = self.sample_random_configuration()
            near_nodes = self.find_near_nodes(q_rand)
            for q_near in near_nodes:
                if self.connect(q_near, q_rand) and self.is_collision_free(q_near, q_rand):
                    if q_near not in self.edges:
                        self.edges[q_near] = []
                    self.edges[q_near].append(q_rand)

    def visualize_roadmap_growth(self):
        plt.figure(figsize=(8, 8))
        for node in self.nodes:
            plt.plot(node[0], node[1], 'bo')  # Plot nodes

            if node in self.edges:
                for neighbor in self.edges[node]:
                    plt.plot([node[0], neighbor[0]], [node[1], neighbor[1]], 'b-')  # Plot edges

        for obstacle in self.obstacles:
            obstacle = np.array(obstacle)
            plt.plot(obstacle[:, 0], obstacle[:, 1], 'r-')  # Plot obstacles

        plt.plot(self.start[0], self.start[1], 'go', markersize=8)  # Start configuration
        plt.plot(self.goal[0], self.goal[1], 'ro', markersize=8)  # Goal configuration

        plt.title('PRM Roadmap Growth')
        plt.xlabel('Configuration Space: Joint 1')
        plt.ylabel('Configuration Space: Joint 2')
        plt.grid(True)
        plt.show()

    def sample_random_configuration(self):
        if random.random() < 0.05:  # Use the goal configuration occasionally
            return self.goal
        return [random.uniform(0, 2 * np.pi), random.uniform(0, 2 * np.pi)]
    
    def find_near_nodes(self, q_rand):
        distances = []
        for node in self.nodes:
            distances.append((np.linalg.norm(np.array(q_rand) - np.array(node)), node))
        distances.sort(key=lambda x: x[0])
        return [node for (_, node) in distances[:self.k]]
    
    def ccw(self, A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    def intersect(self, line1_start, line1_end, line2_start, line2_end):
        return self.ccw(line1_start, line2_start, line2_end) != self.ccw(line1_end, line2_start, line2_end) and self.ccw(line1_start, line1_end, line2_start) != self.ccw(line1_start, line1_end, line2_end)

    def intersects_obstacle(self, line_start, line_end, obstacle):
        for i in range(len(obstacle) - 1):
            if self.intersect(line_start, line_end, obstacle[i], obstacle[i + 1]):
                return True
        return False
    
    def is_collision_free(self, q1, q2):
        # Check if the edge between q1 and q2 is collision-free
        for obstacle in self.obstacles:
            if self.intersects_obstacle(q1, q2, obstacle):
                return False
        return True
    
    def connect(self, q_near, q_rand):
        if q_near not in self.edges:
            self.edges[q_near] = []

        if self.is_collision_free(q_near, q_rand):
            self.edges[q_near].append(q_rand)
            return True
        return False

    def find_path(self):
        queue = [self.start]
        visited = {self.start: None}

        while queue:
            current = queue.pop(0)

            if tuple(current) == self.goal:
                path = [tuple(current)]
                while visited[tuple(current)]:
                    path.append(visited[tuple(current)])
                    current = visited[tuple(current)]
                return path[::-1]

            for neighbor in self.edges.get(tuple(current), []):
                if tuple(neighbor) not in visited:
                    visited[tuple(neighbor)] = tuple(current)
                    queue.append(neighbor)

        return None  # No valid path found

    
    def visualize_solution_path(self, path):
        if path is None:
            print("No valid path found. Unable to visualize.")
            return

        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim([0, 2])
        ax.set_ylim([0, 2])

        for obstacle in self.obstacles:
            obstacle_polygon = plt.Polygon(obstacle, closed=True, fill=None, edgecolor='red')
            ax.add_patch(obstacle_polygon)

        link_arm = LinkArm(link_lengths=[0.4, 0.25], joint_angles=self.start, joint_radius=0.05, link_width=0.1, color='blue', ax=ax, fig=fig)
        plt.plot(self.start[0], self.start[1], 'go', markersize=8)
        plt.plot(self.goal[0], self.goal[1], 'ro', markersize=8)

        def update(frame):
            nonlocal link_arm
            link_arm.update_joints(path[frame])
            link_arm.plot
            return link_arm.plot(),

        ani = animation.FuncAnimation(fig, update, frames=len(path), repeat=False)

        plt.title('PRM Solution Path')
        plt.grid()
        plt.show()

    

    

    
    


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="PRM for 2-link arm motion planning with obstacles.")
    parser.add_argument("--start", nargs=2, type=float, help="Start configuration (joint angles)")
    parser.add_argument("--goal", nargs=2, type=float, help="Goal configuration (joint angles)")
    parser.add_argument("--map", type=str, help="File containing map data (e.g., 'arm_polygons.npy')")
    args = parser.parse_args()

    start = args.start
    goal = args.goal
    map_filename = args.map

    # Load obstacles from the map file
    obstacles = np.load(map_filename, allow_pickle=True)

    # Create PRM instance and perform the search
    prm = PRM(start, goal, obstacles)
    prm.generate_roadmap()
    path = prm.find_path()

    if path:
        prm.visualize_roadmap_growth()
        prm.visualize_solution_path(path)
    else:
        print("No valid path found.")

if __name__ == "__main__":
    main()





