"""
This is the evader class. It handles the evader's position, speed, and movement logic in a pursuit-evasion game.
It has the functions - 
Update position, get position, and calculate heading velocity based on the game state
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar
import time
import Pursuer

class Evader:
    def __init__(self, init_pos, speed, index):
        """Constructor, assigns initial values"""
        self.position = np.array(init_pos).reshape(-1, 1) if np.array(init_pos).ndim == 1 else np.array(init_pos)
        self.speed = speed
        self.index = index
        self.name = f"evader{index}"
    
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
        """Calculate heading velocity based on intercept position and game state"""
        intercept_position = np.array(intercept_position).reshape(-1, 1)
        velocity = intercept_position - self.position
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > 0:
            velocity = (self.speed / velocity_norm) * velocity
        return velocity
    
