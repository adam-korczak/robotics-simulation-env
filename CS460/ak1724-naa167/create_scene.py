import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial import ConvexHull

def random_convex_polygon(n_points, center, min_radius, max_radius):
    angles = np.sort(2 * np.pi * np.random.rand(n_points))
    radii = np.random.uniform(min_radius, max_radius, n_points)
    x = center[0] + radii * np.cos(angles)
    y = center[1] + radii * np.sin(angles)

    hull = ConvexHull(np.column_stack((x, y)))
    return hull.points[hull.vertices]

def random_color(existing_colors):
    while True:
        color = (np.random.rand(), np.random.rand(), np.random.rand())
        if color not in existing_colors:
            existing_colors.add(color)
            return color

def plot_polygons(polygons):
    fig, ax = plt.subplots(dpi=150)
    ax.set_xlim((0, 2))
    ax.set_ylim((0, 2))
    ax.set_aspect('equal', 'box')
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)

    used_colors = set()

    for polygon in polygons:
        color = random_color(used_colors)
        polygon_patch = patches.Polygon(polygon, closed=True, fill=True, edgecolor='black', facecolor=color, alpha=0.6)
        ax.add_patch(polygon_patch)

    plt.show()

def generate_random_obstacles(n_obstacles, n_points_range, radius_range):
    polygons = []
    for _ in range(n_obstacles):
        n_points = np.random.randint(n_points_range[0], n_points_range[1]+1)
        center = np.random.uniform(0, 2, 2)
        polygons.append(random_convex_polygon(n_points, center, radius_range[0], radius_range[1]))
    return polygons

def save_scene(polygons, filename='scene.npy'):
    np.save(filename, polygons)

def load_scene(filename='scene.npy'):
    return np.load(filename, allow_pickle=True)

if __name__ == "__main__":
    option = input("Do you want to generate a new scene or load an existing one? (Enter 'generate' or 'load'): ")

    if option == 'generate':
        n_obstacles = int(input("Enter the total number of polygons in the scene: "))
        min_vertices = int(input("Enter the minimum number of vertices for a polygon: "))
        max_vertices = int(input("Enter the maximum number of vertices for a polygon: "))
        min_radius = float(input("Enter the minimum radius for a polygon (in meters): "))
        max_radius = float(input("Enter the maximum radius for a polygon (in meters): "))

        obstacles = generate_random_obstacles(n_obstacles, (min_vertices, max_vertices), (min_radius, max_radius))
        
        save_option = input("Do you want to save the generated scene? (yes/no): ")
        if save_option == 'yes':
            filename = input("Enter the filename to save the scene (default is 'scene.npy'): ") or 'scene.npy'
            save_scene(obstacles, filename)

        plot_polygons(obstacles)
    
    elif option == 'load':
        filename = input("Enter the filename to load the scene from (default is 'scene.npy'): ") or 'scene.npy'
        obstacles = load_scene(filename)
        plot_polygons(obstacles)
