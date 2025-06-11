from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class World3D:
    """Container for rendering multiple Object3D instances."""

    def __init__(self):
        self.objects = []

    def add(self, obj):
        self.objects.append(obj)

    def draw(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        for obj in self.objects:
            obj.draw(ax)
        plt.show()
