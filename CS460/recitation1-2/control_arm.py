import numpy as np
import matplotlib.pyplot as plt

class RobotArm:
    def __init__(self, ax):
        # Initial joint angles
        self.theta1, self.theta2 = 0, 0

        # Arm lengths
        self.L1, self.L2 = 1, 0.75

        # Plot the robot arm
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.set_aspect('equal')
        self.ax = ax
        self.fig = ax.figure
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.plot_arm()

    def plot_arm(self):
        # Calculate joint and end effector positions
        x1 = self.L1 * np.cos(self.theta1)
        y1 = self.L1 * np.sin(self.theta1)
        x2 = x1 + self.L2 * np.cos(self.theta1 + self.theta2)
        y2 = y1 + self.L2 * np.sin(self.theta1 + self.theta2)

        # Plot the robot arm
        self.ax.clear()
        self.ax.plot([0, x1, x2], [0, y1, y2], '-o', markersize=10)
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-2, 2)
        self.fig.canvas.draw()

    def on_key_press(self, event):
        step = np.pi/30  # Angle step
        if event.key == '1':
            self.theta1 += step
        elif event.key == '4':
            self.theta1 -= step
        elif event.key == '2':
            self.theta2 += step
        elif event.key == '5':
            self.theta2 -= step
        self.plot_arm()

fig, ax = plt.subplots()
arm = RobotArm(ax)
plt.show()