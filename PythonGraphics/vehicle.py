import numpy as np
from .object3d import Object3D
from .boxel import Boxel

class Vehicle(Object3D):
    """Base class for vehicles composed of boxels."""

    def __init__(self, length, width, height, color='blue'):
        super().__init__()
        body = Boxel(center=(0, 0, height/2), size=(length, width, height), color=color)
        self.add_boxel(body)
        self.length = length
        self.width = width
        self.height = height
