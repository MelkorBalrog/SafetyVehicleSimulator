import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class Boxel:
    """Represents a single cuboid primitive."""

    def __init__(self, center, size, color='b'):
        self.center = np.asarray(center, dtype=float)
        self.size = np.asarray(size, dtype=float)
        self.color = color

    def vertices(self):
        cx, cy, cz = self.center
        sx, sy, sz = self.size / 2
        # 8 vertices of the cuboid
        return np.array([
            [cx - sx, cy - sy, cz - sz],
            [cx + sx, cy - sy, cz - sz],
            [cx + sx, cy + sy, cz - sz],
            [cx - sx, cy + sy, cz - sz],
            [cx - sx, cy - sy, cz + sz],
            [cx + sx, cy - sy, cz + sz],
            [cx + sx, cy + sy, cz + sz],
            [cx - sx, cy + sy, cz + sz],
        ])

    def faces(self):
        v = self.vertices()
        return [v[[0,1,2,3]], v[[4,5,6,7]], v[[0,1,5,4]],
                v[[2,3,7,6]], v[[1,2,6,5]], v[[0,3,7,4]]]

    def draw(self, ax):
        faces = self.faces()
        poly3d = Poly3DCollection(faces, facecolors=self.color, linewidths=0.5, edgecolors='k', alpha=0.7)
        ax.add_collection3d(poly3d)
