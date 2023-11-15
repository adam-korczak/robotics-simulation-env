import numpy as np
import argparse
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from rigid_body_car import RigidBodyRobot  

def load_polygons(file_path):
    """ Load the polygon data from a .npy file """
    return np.load(file_path, allow_pickle=True)

def plot_polygons(polygons, ax):
    """ Plot the polygons on the given axis """
    for polygon in polygons:
        patch = patches.Polygon(polygon, closed=True, color='gray', alpha=0.5)
        ax.add_patch(patch)

def collision_free(node, polygons):
    """ Check if the node is collision-free """
    # Implement collision checking between the node and the polygons
    pass

def nearest_node(node, nodes):
    """ Find the nearest node in the tree """
    nodes = np.asarray(nodes)
    dist = np.linalg.norm(nodes - node, axis=1)
    nearest_idx = np.argmin(dist)
    return nodes[nearest_idx]

def rrt(start, goal, polygons, iter_max=500):
    """ RRT pathfinding algorithm """
    nodes = [start]
    for _ in range(iter_max):
        rand = np.random.rand(3)
        rand[0] *= 2  # x coordinate
        rand[1] *= 2  # y coordinate
        rand[2] *= np.pi  # theta
        nearest = nearest_node(rand, nodes)

        # Simple straight-line path towards the random node
        new_node = nearest + 0.05 * (rand - nearest)
        if collision_free(new_node, polygons):
            nodes.append(new_node)
            if np.linalg.norm(new_node - goal) < 0.1:
                return nodes  # Goal reached

    return nodes  # Return the path found so far

def update(frame, robot, path, ax, polygons):
    """ Update function for animation """
    ax.clear()
    plot_polygons(polygons, ax)
    ax.set_xlim(0, 2)
    ax.set_ylim(0, 2)
    ax.set_aspect('equal', adjustable='box')
    config = path[frame]
    robot.set_position(*config)
    robot.draw(ax)

def main():
    parser = argparse.ArgumentParser(description='Animate robot moving using RRT.')
    parser.add_argument('--start', nargs=3, type=float, help='Start configuration [x, y, theta]')
    parser.add_argument('--goal', nargs=3, type=float, help='Goal configuration [x, y, theta]')
    parser.add_argument('--map', type=str, help='Path to map file with polygons')
    args = parser.parse_args()

    start = np.array(args.start)
    goal = np.array(args.goal)
    map_file = args.map

    polygons = load_polygons(map_file)
    robot = RigidBodyRobot()

    path = rrt(start, goal, polygons)

    fig, ax = plt.subplots()
    ani = animation.FuncAnimation(fig, update, frames=len(path), fargs=(robot, path, ax, polygons), interval=50)
    
    plt.show()

if __name__ == "__main__":
    main()
