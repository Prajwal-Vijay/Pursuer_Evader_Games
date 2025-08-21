"""
This is the Pursuer class.
It has the functions - update position, get position, heading velocity, 
"""

import numpy as np


class Pursuer:
    def __init__(self, init_pos, speed, index):
        self.position = init_pos.reshape(-1, 1) if np.array(init_pos).ndim == 1 else np.array(init_pos)
        self.speed = speed
        self.index = index
        self.name = f"pursuer{index}"
        self.capture_radius = 0.1
    
    def update_pos(self, position):
        """Updates position"""
        position = np.array(position)
        if position.shape != self.position.shape:
            if position.T.shape == self.position.shape:
                self.position = position.T
            else:
                raise ValueError('Wrong position dimensions.')
        else:
            self.position = position
    
    def get_pos(self):
        return self.position
    
    def heading_velocity(self, intercept_position):
        """Calculate heading velocity based on intercept position"""
        """Remember the velocity is a vector in 3D space."""
        intercept_position = np.array(intercept_position).reshape(-1, 1)
        velocity = intercept_position - self.position
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > 0:
            velocity = (self.speed / velocity_norm) * velocity
        return velocity