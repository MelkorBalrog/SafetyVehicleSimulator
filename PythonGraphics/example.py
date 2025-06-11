"""Example usage of the Python graphics module."""
from .animation import SimulationAnimator

if __name__ == '__main__':
    anim = SimulationAnimator('Simulations/trajectory.csv')
    anim.animate()
