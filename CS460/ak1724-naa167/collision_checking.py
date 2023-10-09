import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
from create_scene import load_scene, plot_polygons

def check_collision(polygons):
    collision_flags = [False] * len(polygons)
    for i in range(len(polygons)):
        for j in range(len(polygons)):
            if i != j:
                path = Path(polygons[j])
                for point in polygons[i]:
                    if path.contains_point(point):
                        collision_flags[i] = True
                        collision_flags[j] = True
    return collision_flags

def plot_colliding_polygons(polygons):
    fig, ax = plt.subplots(dpi=100)
    ax.set_xlim((0, 2))
    ax.set_ylim((0, 2))
    ax.set_aspect('equal', 'box')
    ax.grid(True, which='both', linestyle='--', linewidth=0.5)

    used_colors = set()

    collision_flags = check_collision(polygons)

    for i, polygon in enumerate(polygons):
        color = 'red' if collision_flags[i] else 'black'
        polygon_patch = patches.Polygon(polygon, closed=True, fill=True, edgecolor=color, alpha=0.6)
        ax.add_patch(polygon_patch)

    plt.show()

if __name__ == "__main__":
    filename = input("Enter the filename to load the scene from (default is 'scene.npy'): ") or 'scene.npy'
    polygons = load_scene(filename, allow_pickle=True)
    #for TAs
    #polygons = np.load(’collision_checking_polygons.npy’, allow_pickle=True)
    plot_colliding_polygons(polygons)


