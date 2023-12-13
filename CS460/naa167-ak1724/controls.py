import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D




# Initial state [x, y, theta]
q = np.array([1.0, 1.0, 0.0])

# Control input [v_R, v_L]
u = np.array([0.0, 0.0])

# Distance between wheels and robot dimensions
L = 0.2
length = 0.1
width = 0.2

# Time step
dt = 0.1

num_steps = 200
# Control limits
v_max = 0.5
v_min = -0.5
omega_max = 0.9
omega_min = -0.9

controls = np.zeros((num_steps, 2))
path = np.zeros((num_steps, 2))

def randomize_q(q):
    q = np.array([np.random.uniform(0.2, 1.8), np.random.uniform(0.2, 1.8), np.random.uniform(0, 2 * np.pi)])
    return q
def differential_drive_model(q, u, L, ctrl_input):
    dq = np.zeros_like(q)
    
    V = ctrl_input[0]
    omega = ctrl_input[1]
    
    dq[0] = V * np.cos(q[2]) * dt
    dq[1] = V * np.sin(q[2]) * dt
    dq[2] = omega * dt
    
    return dq


def draw_rotated_rectangle(ax, center, width, height, angle_degrees, color='b'):
    angle_radians = np.radians(angle_degrees)
    x, y = center
    
    rect = patches.Rectangle((x - width / 2, y - height / 2), width, height, linewidth=1, edgecolor=color, facecolor='none')
    t = Affine2D().rotate_deg_around(x, y, angle_degrees) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)


def update_visual(q, u, ax):
    
    # Draw robot body as a rectangle
    draw_rotated_rectangle(ax, [q[0], q[1]], length, width, np.degrees(q[2]))

    
    # Draw wheels as rectangles
    wheel_width = 0.02
    wheel_length = 0.08
    # Calculate wheel centers
    wheel1_center = [q[0] + (L/2) * np.sin(q[2]), q[1] - (L/2) * np.cos(q[2])]
    wheel2_center = [q[0] - (L/2) * np.sin(q[2]), q[1] + (L/2) * np.cos(q[2])]
    draw_rotated_rectangle(ax, wheel1_center, wheel_length, wheel_width, np.degrees(q[2]), color='r')
    draw_rotated_rectangle(ax, wheel2_center, wheel_length, wheel_width, np.degrees(q[2]), color='r')
    plt.pause(0.08)

def load_landmarks(map_name):
     return np.load(f'naa167-ak1724/maps/{map_name}.npy', allow_pickle=True)

def set_plot(ax):
    ax.set_xlim([0,2])
    ax.set_ylim([0,2])
    plt.grid(True)
    ax.set_aspect('equal')
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='o', label='Landmarks')
    plt.scatter(path[:, 0], path[:, 1], marker='o', color='blue', linewidths=0.5)

def simulate_motion(q,u,ax):
    q = randomize_q(q)
    original = [q[0],q[1],q[2]]
    #print(f"q = {q[0]}, {q[1]}")
    for i in range(10):
        u[0] = np.clip(np.random.uniform(v_min, v_max), v_min, v_max)
        u[1] = np.clip(np.random.uniform(omega_min, omega_max), omega_min, omega_max)
        #print(f"sampled q: {q}")
        for j in range(20):
            test = differential_drive_model(q, u, L, (u[0],u[1]))
            q += test
            x,y,theta = q
            if x<=1.8 and x>=0.2 and y<=1.8 and y>=0.2:
                #print(f"valid config:{x}, {y}")
                controls[(i*20) + j] =[u[0], u[1]]
                q = x, y, theta
            else:
                #print(f"not in range, resampling")
                q = randomize_q(q)
                return simulate_motion(q,u,ax)
                

    q = [original[0], original[1], original[2]]
    count = 0
    for j in range(num_steps):
        #print(f"u({j}) = {u[0]}, {u[1]}")
        u[0], u[1] = controls[j]
        #print(f"u({j}) = {u[0]}, {u[1]}")
        dq = differential_drive_model(q, u, L, controls[j])
        q += dq
        #print(f"x:{q[0]}, y:{q[1]}")
        path[j] = [q[0],q[1]]
        #if q[0]<0.2 or q[0] >1.8 or q[1]<0.2 or q[1]>1.8:
            #print("Collision detected: resampling...")
            #return simulate_motion(q, u, ax)
        update_visual(q,u,ax)
        count += 1
        ax.clear()
        set_plot(ax)
        
    return original
    #print(count)


#dq = differential_drive_model(q, u, L)
for i in range(5):
    for j in range(2):
        controls = np.zeros((num_steps, 2))
        path = np.zeros((num_steps, 2))
        landmarks = load_landmarks(f'landmark_{i}')
        fig, ax = plt.subplots()
        ax.set_aspect('equal')
        ax.set_xlim([0, 2])
        ax.set_ylim([0, 2])
        plt.grid(True)
        plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='o', label='Landmarks')
        initial_pose = simulate_motion(q, u, ax)
        ax.clear()
        output_file = f'controls/controls_{i}_{j}.npy'
        np.save(output_file, np.array([initial_pose, controls], dtype=object))
