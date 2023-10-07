import matplotlib.pyplot as plt

class PointController:
    def __init__(self, ax):
        # Initial position of the point
        self.point, = ax.plot(0.5, 0.5, 'ro')  
        self.x, self.y = 0.5, 0.5
        self.ax = ax
        self.fig = ax.figure
        # Set the axis limits
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        # Connect the event to the callback function
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

    def on_key_press(self, event):
        # Define step size for arrow key movement
        step = 0.05
        if event.key == 'up':
            self.y += step
        elif event.key == 'down':
            self.y -= step
        elif event.key == 'right':
            self.x += step
        elif event.key == 'left':
            self.x -= step
        # Update the point's position
        self.point.set_data([self.x], [self.y])
        self.fig.canvas.draw()

# Create a new figure and axis
fig, ax = plt.subplots()
controller = PointController(ax)
plt.show()