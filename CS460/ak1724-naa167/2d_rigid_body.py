import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
from create_scene import load_scene, plot_polygons

polygons = np.load('scene.npy', allow_pickle=True)

def check_freespace(polygons, rect_vertices):
    #check_collisoin_path = [False] * len(polygons)
    for i in range(len(polygons)):
        path = Path(rect_hull)
        for point in polygons[i]:
            if path.contains_point(point):
                return False
    return True 

def get_rect_vertices(x1, y1):
    x2, y2 = x1, y1 + 0.2
    x3, y3 = x1 + 0.1, y2
    x4, y4 = x3, y1
    rect_vertices = [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
    return rect_vertices  

x1 = np.random.uniform(0.1, 2.0)
y1 = np.random.uniform(0.1, 2.0)
rect_vertices = get_rect_vertices(x1, y1)
rect_hull = ConvexHull(rect_vertices)
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(0.0, 2.0)
ax.set_ylim(0.0, 2.0)
ax.set_aspect('equal', 'box')

while not(check_freespace(polygons, rect_vertices)):
    x1 = np.random.uniform(0.1, 2.0)
    y1 = np.random.uniform(0.1, 2.0)
    rect_vertices = get_rect_vertices(x1, y1)
    
polygons.append(rect_hull.points[rect_hull.vertices])
rect_patch = patches.Polygon(rect_vertices, closed=True, fill=True)
ax.add_patch(rect_patch)


def rotate(vertices, angle_degrees, center_point):
    angle_rad = np.deg2rad(angle_degrees)
    rot_matrix = np.array([[np.cos(angle_rad), -np.sin(angle_rad)],
                           [np.sin(angle_rad),  np.cos(angle_rad)]])

    vertices_rotated = (vertices - center_point).dot(rot_matrix) + center_point
    return vertices_rotated

def on_key(event):
    global rect_vertices
    center = np.mean(rect_vertices, axis=0)  # Center of the rectangle
    if event.key == '1':
        # Move forward (upward in this view)
        rect_vertices += [0, 0.1]
    elif event.key == '2':
        # Move backward (downward in this view)
        rect_vertices -= [0, 0.1]
    elif event.key == '3':
        # Rotate counter-clockwise
        rect_vertices = rotate(rect_vertices, 5, center)
    elif event.key == '4':
        # Rotate clockwise
        rect_vertices = rotate(rect_vertices, -5, center)
    else:
        return
    rect_patch.set_xy(rect_vertices)
    fig.canvas.draw()

fig.canvas.mpl_connect('key_press_event', on_key)
create_scene.plot_polygons(polygons)
plt.show()
