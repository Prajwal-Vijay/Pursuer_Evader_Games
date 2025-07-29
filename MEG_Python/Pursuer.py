import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar
import time
import Evader

class Pursuer:
    def __init__(self, init_pos, speed):
        """Constructor, assigns initial values"""
        self.position = np.array(init_pos).reshape(-1, 1) if np.array(init_pos).ndim == 1 else np.array(init_pos)
        self.speed = speed
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
    
    def optimal_headings_Ei(self, evader_position, target_position):
        """Calculate optimal headings for evader i"""
        evader_position = np.array(evader_position).reshape(-1, 1)
        target_position = np.array(target_position).reshape(-1, 1)
        
        if self.position[0, 0] != evader_position[0, 0]:
            m = (evader_position[1, 0] - self.position[1, 0]) / (evader_position[0, 0] - self.position[0, 0])
        else:
            m = float('inf')
        
        xm = (evader_position[0, 0] + self.position[0, 0]) / 2
        ym = (evader_position[1, 0] + self.position[1, 0]) / 2
        xt, yt = target_position[0, 0], target_position[1, 0]
        
        if m != float('inf'):
            xI = (m * (ym - yt) + xm + m**2 * xt) / (1 + m**2)
            yI = (m * (xm - xt) + yt + m**2 * ym) / (1 + m**2)
        else:
            xI = xm
            yI = yt
        
        psi_star = np.arctan2(yI - evader_position[1, 0], xI - evader_position[0, 0])
        theta_star = np.arctan2(yI - self.position[1, 0], xI - self.position[0, 0])
        
        return psi_star, theta_star, m, xm, ym
    
    def objective_Ei(self, evader_position, target_position, r, theta):
        """Objective function for evader i"""
        psi_star, _, m, xm, ym = self.optimal_headings_Ei(evader_position, target_position)
        evader_position = np.array(evader_position).reshape(-1, 1)
        target_position = np.array(target_position).reshape(-1, 1)
        
        xt, yt = target_position[0, 0], target_position[1, 0]
        
        if m != float('inf'):
            Tx = abs(m * yt + xt - m * ym - xm) / np.sqrt(1 + m**2)
            Ty = abs(yt - m * xt - evader_position[1, 0] + m * evader_position[0, 0]) / np.sqrt(1 + m**2)
        else:
            Tx = abs(xt - xm)
            Ty = abs(yt - evader_position[1, 0])
        
        k = np.linalg.norm(self.position - evader_position) / 2
        delta = np.arctan2(self.position[1, 0] - evader_position[1, 0], 
                          self.position[0, 0] - evader_position[0, 0])
        
        if k > 0:
            d_i = Tx + (r / (2 * k)) * np.sqrt(Ty**2 + k**2) * (-1 - np.cos(theta + psi_star - 2 * delta))
        else:
            d_i = Tx
        
        return d_i
    
    def concave_domain(self, evader_positions, target_position):
        """Calculate concave domain for heading angles"""
        n = evader_positions.shape[1]
        theta_values = np.zeros(n)
        
        for i in range(n):
            _, theta_star = self.optimal_headings_Ei(evader_positions[:, i:i+1], target_position)
            theta_values[i] = theta_star
        
        sort_order = np.argsort(theta_values)
        theta_values = theta_values[sort_order]
        
        max_idx = np.argmax(theta_values)
        min_idx = np.argmin(theta_values)
        
        theta_largest = theta_values[max_idx]
        theta_smallest = theta_values[min_idx]
        
        theta_min = max(theta_smallest, theta_largest - np.pi/2)
        theta_max = min(theta_largest, theta_smallest + np.pi/2)
        
        return theta_min, theta_max, sort_order[min_idx], sort_order[max_idx]
    
    def objective_fun(self, evader_positions, target_position, r, theta):
        """Main objective function"""
        n = evader_positions.shape[1]
        cost = 0
        for i in range(n):
            obj_val = self.objective_Ei(evader_positions[:, i:i+1], target_position, r, theta)
            if obj_val != 0:
                cost += 1 / obj_val
        return cost
    
    def heading_direction_heuristic(self, evader_positions, target_position, r):
        """Heuristic heading direction"""
        n = evader_positions.shape[1]
        p_values = np.zeros(n)
        theta_star = np.zeros(n)
        
        for i in range(n):
            _, theta_star[i], _,_,_ = self.optimal_headings_Ei(evader_positions[:, i:i+1], target_position)
        
        for i in range(n):
            p_values[i] = self.objective_Ei(evader_positions[:, i:i+1], target_position, r, theta_star[i])
        
        if np.sum(p_values) > 0:
            weights = p_values / np.sum(p_values)
            theta = np.dot(weights, theta_star)
        else:
            theta = np.mean(theta_star)
        
        return theta
    
    def heading_direction_closest(self, evader_positions):
        """Head towards closest evader"""
        n = evader_positions.shape[1]
        distances = np.zeros(n)
        
        for i in range(n):
            distances[i] = np.linalg.norm(self.position - evader_positions[:, i:i+1])
        
        closest_idx = np.argmin(distances)
        theta = np.arctan2(evader_positions[1, closest_idx] - self.position[1, 0],
                          evader_positions[0, closest_idx] - self.position[0, 0])
        return theta
    
    def heading_velocity(self, evader_positions, target_position, r, win, objective_function):
        """Calculate heading velocity"""
        if win:
            if objective_function == 'heuristic':
                theta = self.heading_direction_heuristic(evader_positions, target_position, r)
            elif objective_function == 'closest':
                theta = self.heading_direction_closest(evader_positions)
            else:
                print(f'Unknown objective function: {objective_function}. Using heuristic.')
                theta = self.heading_direction_heuristic(evader_positions, target_position, r)
            
            velocity = self.speed * np.array([[np.cos(theta)], [np.sin(theta)]])
        else:
            target_position = np.array(target_position).reshape(-1, 1)
            velocity = target_position - self.position
            velocity_norm = np.linalg.norm(velocity)
            if velocity_norm > 0:
                velocity = (self.speed / velocity_norm) * velocity
            theta = np.arctan2(velocity[1, 0], velocity[0, 0])
        
        return velocity, theta
