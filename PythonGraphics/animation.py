import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from .world import World3D
from .truck import Truck

class SimulationAnimator:
    """Animate simulation results using matplotlib."""

    def __init__(self, csv_file):
        self.csv_file = csv_file
        self.world = World3D()
        self.vehicle = Truck()
        self.world.add(self.vehicle)

    def load_data(self):
        with open(self.csv_file, 'r') as f:
            reader = csv.DictReader(f)
            self.frames = [row for row in reader]

    def animate(self):
        self.load_data()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for frame in self.frames:
            ax.cla()
            x = float(frame.get('x', 0))
            y = float(frame.get('y', 0))
            yaw = float(frame.get('yaw', 0))
            self.vehicle.position = np.array([x, y, 0])
            c = np.cos(yaw)
            s = np.sin(yaw)
            self.vehicle.orientation = np.array([[c,-s,0],[s,c,0],[0,0,1]])
            self.vehicle.draw(ax)
            ax.set_xlim(-10, 10)
            ax.set_ylim(-10, 10)
            ax.set_zlim(0, 5)
            plt.pause(0.01)
        plt.show()
