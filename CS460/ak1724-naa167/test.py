import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
def random_polygon(n_points, xlim, ylim):
    """
    Generates random polygon with n_points points within xlim and ylim.
    """
    x = np.random.uniform(xlim[0], xlim[1], n_points)
    y = np.random.uniform(ylim[0], ylim[1], n_points)
    return np.column_stack((x, y))
def plot_polygons(polygons, xlim, ylim):
    """
    Plots polygons within given xlim and ylim.
    """
    fig, ax = plt.subplots()
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_aspect('equal', 'box')
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)
    
    for polygon in polygons:
        polygon_patch = patches.Polygon(polygon, closed=True, fill=True, edgecolor='black', alpha=0.6)
        ax.add_patch(polygon_patch)

    plt.show()

def generate_random_obstacles(n_obstacles, n_points_range, xlim, ylim):
    """
    Generates random obstacles each having random number of points within n_points_range.
    """
    polygons = []
    for _ in range(n_obstacles):
        n_points = np.random.randint(n_points_range[0], n_points_range[1]+1)
        polygons.append(random_polygon(n_points, xlim, ylim))
    return polygons

# Parameters
xlim = (0, 10)
ylim = (0, 10)
n_obstacles = 5
n_points_range = (3, 6)  # Polygons will have between 3 and 6 points

# Generate and plot obstacles
obstacles = generate_random_obstacles(n_obstacles, n_points_range, xlim, ylim)
plot_polygons(obstacles, xlim, ylim)