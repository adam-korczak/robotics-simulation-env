import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Rectangle

map_polygons = np.load("arm_polygons.npy", allow_pickle=True)
class NLinkArm(object):
    """
    Class for controlling and plotting a planar arm with an arbitrary number of links.
    """

    def __init__(self, link_lengths, joint_angles, joint_radius, link_width):
        self.n_links = len(link_lengths)
        if self.n_links != len(joint_angles):
            raise ValueError()

        self.link_lengths = np.array(link_lengths)
        self.joint_angles = np.array(joint_angles)
        self.joint_radius = joint_radius
        self.link_width = link_width
        self.points = np.ones((self.n_links + 1, 2))

        self.terminate = False
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim([0, 2])
        self.ax.set_ylim([0, 2])
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        self.update_points()
        self.plot()

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
    

    def rotate_joint(self, joint_idx, direction):
        """Rotate joint by a given direction. Positive for counterclockwise, negative for clockwise."""
        delta_angle = 5 * np.pi / 180  # 5 degrees in radians
        self.joint_angles[joint_idx] += delta_angle * direction
        self.update_points()

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
        self.ax.clear()

        for i in range(self.n_links):
            rectangle = self.draw_rectangle(self.points[i], self.points[i + 1])
            self.ax.plot(rectangle[:, 0], rectangle[:, 1], 'orange')
            self.ax.fill(rectangle[:, 0], rectangle[:, 1], 'orange', alpha=0.3)  # Filling the rectangle

        for i in range(self.n_links + 1):
            circle = patches.Circle(self.points[i], radius=self.joint_radius, facecolor='black')
            self.ax.add_patch(circle)
        self.ax.set_xlim([0, 2])
        self.ax.set_ylim([0, 2])

        for polygon in map_polygons:
            plt.fill(*zip(*polygon), "r")

        plt.draw()
        plt.pause(1e-5)


def point_in_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    in_polygon = False

    p1x, p1y = polygon[0]
    for i in range(n+1):
        p2x, p2y = polygon[i%n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        x_intersection = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x<= x_intersection:
                            in_polygon = not in_polygon
        p1x, p1y = p2x, p2y
    return in_polygon


def collision_free_config(arm_lengths, body_lengths, link_width):
    angles = np.random.rand(2) * 2 * np.pi
    x1 = 1
    y1 = 1
    collides = 0
    while True:
        x2 = x1 + arm_lengths[0] * np.cos(angles[0])
        y2 = y1 + arm_lengths[0] * np.sin(angles[0])
        x3 = x2 + arm_lengths[1] * np.cos(angles[0] + angles[1])
        y3 = y2 + arm_lengths[1] * np.sin(angles[0] + angles[1])

        arm_1 = Rectangle((x1 - body_lengths[0] / 2, y1 - link_width / 2), body_lengths[0], link_width, angle=np.degrees(angles[0]))
        arm_2 = Rectangle((x2 - body_lengths[1] / 2, y2 - link_width / 2), body_lengths[1], link_width, angle=np.degrees(angles[0] + angles[1]))

        for polygon in map_polygons:
            if is_collision(arm_1, polygon) or is_collision(arm_2, polygon):
                collides = 1
                break
        if collides == 0:
            return angles
        else:
            angles = np.random.rand(2) * 2 * np.pi
            continue

def is_collision(rect, polygon):
        rect_points = np.array(rect.get_path().vertices[:-1])
        return any(point_in_polygon(point, polygon) for point in rect_points)


if __name__ == "__main__":    

    angles = collision_free_config((0.4, 0.25), (0.3, 0.15), 0.1)
    arm = NLinkArm([0.4, 0.25],[angles[0], angles[1]], joint_radius=0.05, link_width=0.1)
    
    arm.run()
