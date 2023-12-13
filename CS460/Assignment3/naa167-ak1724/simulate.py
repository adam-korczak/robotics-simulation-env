import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D


L = 0.2
length = 0.1
width = 0.2
dt = 0.1

v_max = 0.5
v_min = -0.5
omega_max = 0.9
omega_min = -0.9



def load_landmarks(map_name):
    return np.load(f'naa167-ak1724/maps/{map_name}.npy', allow_pickle=True)

def set_plot(ax,landmarks):
    ax.set_xlim([0,2])
    ax.set_ylim([0,2])
    plt.grid(True)
    ax.set_aspect('equal')
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='o', label='Landmarks')

def draw_rotated_rectangle(ax, center, width, height, angle_degrees, color='b'):
    angle_radians = np.radians(angle_degrees)
    x, y = center
    
    rect = patches.Rectangle((x - width / 2, y - height / 2), width, height, linewidth=1, edgecolor=color, facecolor='none')
    t = Affine2D().rotate_deg_around(x, y, angle_degrees) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)

def visualize(q, ax):
    draw_rotated_rectangle(ax,[q[0], q[1]], length, width, np.degrees(q[2]))
    wheel_width = 0.02
    wheel_length = 0.08
    wheel1_center = [q[0] + (L/2) * np.sin(q[2]), q[1] - (L/2) * np.cos(q[2])]
    wheel2_center = [q[0] - (L/2) * np.sin(q[2]), q[1] + (L/2) * np.cos(q[2])]
    draw_rotated_rectangle(ax, wheel1_center, wheel_length, wheel_width, np.degrees(q[2]), color='r')
    draw_rotated_rectangle(ax, wheel2_center, wheel_length, wheel_width, np.degrees(q[2]), color='r')
    plt.pause(0.08)

def actuation_noise(initial_pose, controls, linear_noise, angular_noise,act_controls):
    num_steps = len(controls)
    poses = [initial_pose]

    dt = 0.1

    for i in range(num_steps):
        v_planned, omega_planned = controls[i][0], controls[i][0]

        v_noisy = np.clip(v_planned + np.random.normal(0, linear_noise),v_min, v_max)
        omega_noisy = np.clip(omega_planned + np.random.normal(0, angular_noise), omega_min,omega_max)
        act_controls.append([v_noisy, omega_noisy])
        x, y, theta = poses[i]
        x += v_noisy * np.cos(theta) * dt
        y += v_noisy * np.sin(theta) * dt
        theta += omega_noisy * dt

        poses.append([x, y, theta])
        #print(f"u_executed({i}) = {u_noisy[i]}")
    
    return np.array(poses)

def odometry_measurements(ground_truth_poses, linear_noise, angular_noise,act_controls, od_poses):
    num_steps = len(ground_truth_poses)
    measurements = np.zeros((num_steps-1, 2))


    for i in range(num_steps-1):
        v_planned, omega_planned = act_controls[i]

        v_sensed = np.clip(v_planned + np.random.normal(0, linear_noise),v_min, v_max)
        omega_sensed = np.clip(omega_planned + np.random.normal(0, angular_noise), omega_min,omega_max)
        measurements[i] = [v_sensed, omega_sensed]

        x, y, theta = od_poses[i]
        x += v_sensed * np.cos(theta) * dt
        y += v_sensed * np.sin(theta) * dt
        theta += omega_sensed * dt

        od_poses.append([x, y, theta])
            
    return measurements


def landmark_sensor(ground_truth_x, ground_truth_y, ground_truth_theta, landmarks):
    landmarks_local = []
    for landmark in landmarks:
        dx = landmark[0] - ground_truth_x
        dy = landmark[1] - ground_truth_y
        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx) - ground_truth_theta
        landmarks_local.append([distance, angle])
    return np.array(landmarks_local)

def landmark_measurements(poses, landmark_map, distance_noise, angle_noise):
    num_steps = len(poses)
    num_landmarks = len(landmark_map)

    measurements = np.zeros((num_steps, 2*num_landmarks))

    for i in range(num_steps):
        x, y, theta = poses[i]
        landmarks = landmark_sensor(x, y, theta, landmark_map)#ground truth landmarks
        noisy_landmarks = np.zeros_like(landmarks)
        noisy_landmarks[:, 0] = landmarks[:, 0] + np.random.normal(0, distance_noise, num_landmarks)
        noisy_landmarks[:, 1] = landmarks[:, 1] + np.random.normal(0, angle_noise, num_landmarks)

        #measurements[i, :num_landmarks] = noisy_landmarks[:, 0]
        #measurements[i, num_landmarks:] = noisy_landmarks[:, 1]
        measurements[i] = noisy_landmarks.flatten()
        #print(f"measurements: {measurements[i]}")
    return measurements

def main():
    parser = argparse.ArgumentParser(description='Simulate robot motion with noise')
    parser.add_argument('--plan', type=str, help='Path to the control sequence file')
    parser.add_argument('--map', type=str, help='Path to the landmark map file')
    parser.add_argument('--execution', type=str, help='Path to store ground truth poses')
    #parser.add_argument('--sensing', type=str, help='Path to store simulated sensor readings')
    args = parser.parse_args()
    
    control_sequence = np.load(args.plan, allow_pickle=True) 
    landmarks = np.load(args.map, allow_pickle=True)
    num_steps = 200
    num_landmarks = len(landmarks)
    
    initial_pose = control_sequence[0]

    #to store the poses from the odometry model                        
    low_od_poses = [initial_pose]
    high_od_poses = [initial_pose]

    #To store the controls from the actuation model
    act_controls = []

    #FOR DEBUGGING
    count = 0
    controls = []
    for i in range(num_steps):
        controls.append(control_sequence[1][i])

    controls = np.array(controls)
    #LINEAR/ANGULAR NOISE STD FOR ACTUATION
    act_linear_noise = 0.075
    act_angular_noise = 0.2

    #LINEAR/ANGULAR NOISE STD FOR ODOMETRY
    od_low_linear = 0.05
    od_low_angular = 0.1
    od_high_linear = 0.1
    od_high_angular = 0.3
    

    #DISTANCE AND ANGULAR NOISE STD FOR OBSERVATION:
    obs_distance_noise = 0.02
    obs_angular_noise = 0.02

    ground_truth_pose = actuation_noise(initial_pose, controls, act_linear_noise, act_angular_noise,act_controls)
    low_odometry_measurements = odometry_measurements(ground_truth_pose, od_low_linear, od_low_angular, act_controls, low_od_poses)
    high_odometry_measurements = odometry_measurements(ground_truth_pose, od_high_linear, od_high_angular, act_controls, high_od_poses)

    observation_measurements = landmark_measurements(ground_truth_pose, landmarks, obs_distance_noise, obs_angular_noise)
#FOR DISPLAYING
    """fig, ax = plt.subplots()
    ax.set_xlim([0,2])
    ax.set_ylim([0,2])
    plt.grid(True)
    plt.scatter(landmarks[:, 0], landmarks[:, 1], marker='o', label='Landmarks')

    gt_path = np.zeros((num_steps, 2))
    od_path = np.zeros((num_steps, 2))

    for i in range(num_steps):
        gt_path[i] = ground_truth_pose[i][0], ground_truth_pose[i][1]
        plt.scatter(gt_path[:, 0], gt_path[:, 1], marker='o', color='blue', linewidths=0.5)
        
        #switch between high or low for visualization
        od_path[i] = low_od_poses[i][0], low_od_poses[i][1]
        plt.scatter(od_path[:, 0], od_path[:, 1], marker='o', color='red', linewidths=0.5)

        visualize(low_od_poses[i], ax)
        count += 1
        ax.clear()
        set_plot(ax,landmarks)"""
#END OF DISPLAY 
    
    #SAVE THE GROUND TRUTH POSES
    #np.save(args.execution, ground_truth_poses)
    
    #SAVE THE READINGS
    num_poses = len(ground_truth_pose)
    readings = np.zeros((num_poses, 2 + 2 * num_landmarks))
    #print(len(readings))

    low_readings = [initial_pose]
    for i in range(num_poses-1):
        low_readings.append(low_odometry_measurements[i].tolist())
        low_readings.append(observation_measurements[i+1].tolist())

    low_readings_array = np.asarray(low_readings)
    print(low_readings_array)





if __name__ == "__main__":
    main()
