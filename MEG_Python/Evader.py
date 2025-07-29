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
        self.wheel_radius = 0.025
        self.wheel_centre_radius = 0.15
    
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

    def heading_velocity(self, pursuer_position, target_position, win, alpha):
        """Calculate heading velocity based on game state"""
        pursuer_position = np.array(pursuer_position).reshape(-1, 1)
        target_position = np.array(target_position).reshape(-1, 1)
        
        if win and alpha == 1:
            xc = (self.position[0, 0] + pursuer_position[0, 0]) / 2
            yc = (self.position[1, 0] + pursuer_position[1, 0]) / 2
            
            if self.position[0, 0] != pursuer_position[0, 0]:
                m = (self.position[1, 0] - pursuer_position[1, 0]) / (self.position[0, 0] - pursuer_position[0, 0])
                x_intercept = (m * (yc - target_position[1, 0]) + xc + m**2 * target_position[0, 0]) / (1 + m**2)
                y_intercept = target_position[1, 0] + m * (x_intercept - target_position[0, 0])
            else:
                x_intercept = xc
                y_intercept = target_position[1, 0]
            
            velocity = np.array([[x_intercept - self.position[0, 0]], [y_intercept - self.position[1, 0]]])
            
        elif win and alpha < 1:
            xc = (self.position[0, 0] - alpha**2 * pursuer_position[0, 0]) / (1 - alpha**2)
            yc = (self.position[1, 0] - alpha**2 * pursuer_position[1, 0]) / (1 - alpha**2)
            rc = (alpha / (1 - alpha**2)) * np.linalg.norm(pursuer_position - self.position)
            Rc = np.linalg.norm(np.array([[xc], [yc]]) - target_position)
            
            x_intercept = target_position[0, 0] + (1 - (rc / Rc)) * xc
            y_intercept = target_position[1, 0] + (1 - (rc / Rc)) * yc
            
            velocity = np.array([[x_intercept - self.position[0, 0]], [y_intercept - self.position[1, 0]]])
        else:
            velocity = target_position - self.position
        
        velocity_norm = np.linalg.norm(velocity)
        if velocity_norm > 0:
            velocity = (self.speed / velocity_norm) * velocity
        
        psi = np.arctan2(velocity[1, 0], velocity[0, 0])
        return velocity, psi
