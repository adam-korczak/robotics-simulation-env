import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

#this isnt right but good start

def dot(v, w):
    return v[0] * w[0] + v[1] * w[1]

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
       return v
    return v / norm

def project_polygon(axis, polygon):
    """Project the polygon on the given axis."""
    min_proj = float('inf')
    max_proj = float('-inf')
    
    for vertex in polygon:
        proj = dot(vertex, axis)
        min_proj = min(min_proj, proj)
        max_proj = max(max_proj, proj)
        
    return (min_proj, max_proj)

def is_separating_axis(axis, poly1, poly2):
    """Check if the axis is a separating axis."""
    proj1 = project_polygon(axis, poly1)
    proj2 = project_polygon(axis, poly2)
    
    return proj1[1] < proj2[0] or proj2[1] < proj1[0]

def check_collision(poly1, poly2):
    edges = []
    for i in range(len(poly1)):
        edges.append((poly1[i], poly1[(i+1)%len(poly1)]))
    for i in range(len(poly2)):
        edges.append((poly2[i], poly2[(i+1)%len(poly2)]))

    for edge in edges:
        axis = normalize(np.array([edge[1][1] - edge[0][1], edge[0][0] - edge[1][0]]))
        if is_separating_axis(axis, poly1, poly2):
            return False

    return True

def plot_polygons(poly1, poly2):
    fig, ax = plt.subplots()
    poly1_patch = patches.Polygon(poly1, closed=True, edgecolor='blue', facecolor='none')
    poly2_patch = patches.Polygon(poly2, closed=True, edgecolor='red', facecolor='none')
    ax.add_patch(poly1_patch)
    ax.add_patch(poly2_patch)
    
    plt.xlim(-10, 10)
    plt.ylim(-10, 10)
    plt.grid()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

# Example:
poly1 = [[0, 0], [4, 0], [2, 4]]
poly2 = [[3, 1], [7, 1], [5, 5]]

if check_collision(poly1, poly2):
    print("Polygons collide!")
else:
    print("Polygons do not collide!")

plot_polygons(poly1, poly2)
