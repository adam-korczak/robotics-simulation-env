import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def parse_args():
    parser = argparse.ArgumentParser(description='Particle Filter for Robot Localization')
    parser.add_argument('--map', type=str, required=True, help='Path to the landmark map file')
    parser.add_argument('--sensing', type=str, required=True, help='Path to the sensor readings file')
    parser.add_argument('--num_particles', type=int, required=True, help='Number of particles to use in the filter')
    parser.add_argument('--estimates', type=str, required=True, help='Path to store particle filter estimates')
    return parser.parse_args()

def initialize_particles(initial_pose, num_particles):
    particles = np.tile(initial_pose, (num_particles, 1))
    return particles

def apply_motion_model(particle, control):
    x, y, theta = particle
    v, omega = control
    x_new = x + v * np.cos(theta)
    y_new = y + v * np.sin(theta)
    theta_new = theta + omega
    return [x_new, y_new, theta_new]

def calculate_weight(particle, sensor_reading, map_landmarks):
    # Placeholder function for calculating weights
    # Implement the actual sensor model for your scenario
    return np.random.random()

def particle_filter(particles, map_landmarks, sensor_reading):
    control = sensor_reading[:2]  # Assuming control data is in sensor_reading
    particles = np.array([apply_motion_model(p, control) for p in particles])

    weights = np.array([calculate_weight(p, sensor_reading, map_landmarks) for p in particles])
    weights /= np.sum(weights)

    indices = np.random.choice(range(len(particles)), size=len(particles), p=weights)
    particles = particles[indices]

    return particles, weights

def update_estimates(particles, weights):
    mean_estimate = np.average(particles, axis=0, weights=weights)
    return mean_estimate

def plot_map(ax, map_landmarks):
    ax.scatter(map_landmarks[:, 0], map_landmarks[:, 1], marker='o', color='blue', label='Landmarks')

def plot_particles(ax, particles, weights):
    ax.scatter(particles[:, 0], particles[:, 1], alpha=0.5, color='black', label='Particles')

def plot_odometry(ax, sensor_readings, current_step):
    odometry_positions = sensor_readings[:current_step, :2]
    ax.plot(odometry_positions[:, 0], odometry_positions[:, 1], color='red', label='Odometry')

def plot_noisy_observations(ax, sensor_readings, current_step):
    noisy_obs = sensor_readings[current_step][3:]  # Adjust based on your data format
    for i in range(0, len(noisy_obs), 2):
        ax.scatter(noisy_obs[i], noisy_obs[i+1], marker='x', color='red', label='Noisy Observations')

def animate(i, particles, weights, ax, map_landmarks, sensor_readings):
    ax.clear()
    plot_map(ax, map_landmarks)
    plot_particles(ax, particles, weights)
    plot_odometry(ax, sensor_readings, i)
    plot_noisy_observations(ax, sensor_readings, i)

def main():
    args = parse_args()

    map_landmarks = np.load(args.map)
    sensor_readings = np.load(args.sensing)
    initial_pose = sensor_readings[0]
    particles = initialize_particles(initial_pose, args.num_particles)

    fig, ax = plt.subplots()
    anim = FuncAnimation(fig, lambda i: animate(i, particles, weights, ax, map_landmarks, sensor_readings),
                         frames=len(sensor_readings), interval=100)

    estimates = []

    for reading in sensor_readings:
        particles, weights = particle_filter(particles, map_landmarks, reading)
        estimate = update_estimates(particles, weights)
        estimates.append(estimate)

    np.save(args.estimates, np.array(estimates))

    plt.show()
    # Optionally, save the animation using anim.save('animation.mp4')

if __name__ == "__main__":
    main()
