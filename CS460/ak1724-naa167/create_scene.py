import matplotlib.pyplot as plt

class CreateScene:
    def __init__(self, ax):

        self.ax = ax
        self.fig = ax.figure
        # Set the axis limits
        self.ax.set_xlim(0, 2)
        self.ax.set_ylim(0, 2)


# Create a new figure and axis
fig, ax = plt.subplots()
controller = CreateScene(ax)
plt.show()