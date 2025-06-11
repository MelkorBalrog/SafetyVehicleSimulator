import numpy as np
from .boxel import Boxel

class Object3D:
    """Composite 3D object made from Boxels."""

    def __init__(self, boxels=None, position=(0,0,0)):
        self.boxels = list(boxels) if boxels else []
        self.position = np.asarray(position, dtype=float)
        self.orientation = np.eye(3)

    def add_boxel(self, boxel: Boxel):
        self.boxels.append(boxel)

    def transform_vertices(self, verts):
        return (self.orientation @ verts.T).T + self.position

    def draw(self, ax):
        for boxel in self.boxels:
            b = Boxel(self.transform_vertices(boxel.center), boxel.size, boxel.color)
            b.draw(ax)
