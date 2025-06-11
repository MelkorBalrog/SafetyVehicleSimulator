from .vehicle import Vehicle
from .boxel import Boxel

class Truck(Vehicle):
    """Simple tractor + trailer."""

    def __init__(self):
        super().__init__(length=4.0, width=2.0, height=1.5, color='silver')
        trailer = Boxel(center=(6.0, 0, 1.5/2), size=(8.0, 2.5, 1.5), color='gray')
        self.add_boxel(trailer)
