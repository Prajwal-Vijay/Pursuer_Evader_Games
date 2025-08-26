"""
Includes all of the main code for finding the value function, implementing the matching algorithm, and running the simulation.
Functions include: Check initialization, plot current positions, step function, obtain trajectories.
"""
import numpy as np
import matplotlib.pyplot as plt
import Pursuer
import Evader
import time
import random
import cvxpy as cp
from itertools import combinations

class Environment:
    def __init__(self, N, M, t, pursuers, evaders):
        self.N = N  # Number of pursuers
        self.M = M  # Number of evaders
        self.timestep = t  # Time step
        self.pursuers = pursuers  # Assuming a single pursuer for simplicity
        self.evaders = evaders
        

    def check_initialization(self, verbose=False):
        """Check if the initial positions are valid for the simulation"""
        # Check if any evader is already captured or too close to the pursuer
        for pursuer in self.pursuers:
            for evader in self.evaders:
                if np.linalg.norm(pursuer.get_pos() - evader.get_pos()) < pursuer.capture_radius:
                    if verbose:
                        print(f"Evader {evader.index} is too close to the pursuer at initialization.")
                    return False
        for evader in self.evaders:
            if evader.get_pos()[2,0] < 0: # Might be some issues with the way I am accessing here.
                if verbose:
                    print(f"Evader {evader.index} is not in the play region at initialization.")
                return False
        """ 
            When the intersection of evasion space and the goal region is empty, we will have no point in the goal that the evaders can
            reach before getting caught by the pursuers. Paper defines interception point, only in these cases. So we will create
            a matrix of [f_ij(x)], for all the pairs of (evaders) vs (pursuer subcoalitions). Then use pycvx or some optimization problem to figure
            out whether, the min z, is lying below the plane or above the plane z = 0.
            Do we really have to calculate for all the coalitions now itself, or can we consider them seperately and calculate? No we need to include all the 
            pursuers coalitions and try to calculate, because it may happen individually the evader cannot be caught, but as a group it can be.
        """
        evasion_matrix, coalition_list = self.create_evasion_matrix()
        for evader_idx, evader in enumerate(self.evaders):
            for coalition in coalition_list:
                if evasion_matrix[(evader_idx, coalition)] == 1:
                    # There exists atleast one coalition which can successfully capture the evader.
                    break 
            else:
                # There exists an evader that always manages to escape!
                return False
        return True

    def create_evasion_matrix(self, max_coalition_size=3):
        # Generate all possible pursuer coalitions up to max_coalition_size
        coalition_list = []
        for size in range(1, min(max_coalition_size + 1, len(self.pursuers) + 1)):
            for coalition in combinations(range(len(self.pursuers)), size):
                coalition_list.append(coalition)
        
        evasion_matrix = {}
        
        # For each evader and each pursuer coalition
        for evader_idx, evader in enumerate(self.evaders):
            for coalition in coalition_list:
                min_z = self._compute_min_z_in_bes(evader_idx, coalition)
                
                # Set matrix value: 1 if min_z > 0, -1 otherwise
                evasion_matrix[(evader_idx, coalition)] = 1 if min_z > 0 else -1
        return evasion_matrix, coalition_list

    def _compute_min_z_in_bes(self, evader_idx, coalition):
        # Uses the boundary of evasion space method as given in the paper.
        evader = self.evaders[evader_idx]
        x = cp.Variable(3)
        objective = cp.Minimize(x[2])
        constraints = []
        for pursuer_idx in coalition:
            pursuer = self.pursuers[pursuer_idx]
            evader_pos = evader.get_pos()
            pursuer_pos = pursuer.get_pos()
            
            alpha_ij = pursuer.speed/evader.speed
            capture_radius = pursuer.capture_radius
            
            dist_to_pursuer = cp.norm(x - pursuer_pos, 2)
            dist_to_evader = cp.norm(x - evader_pos, 2)
            constraints.append(dist_to_pursuer - alpha_ij*dist_to_evader >= capture_radius)
        problem = cp.Problem(objective, constraints)
        try:
            problem.solve(solver=cp.ECOS, verbose=False)
            
            if problem.status == cp.OPTIMAL:
                return problem.value
            else:
                return -np.inf
        except:
            return -np.inf
            
    def plot_current_positions(self):
        """Plot current positions of pursuers and evaders"""
        import matplotlib.pyplot as plt
        
        plt.figure(figsize=(10, 10))
        plt.plot(self.pursuer.position[0, 0], self.pursuer.position[1, 0], 'ro', label='Pursuer')
        
        for i, evader in enumerate(self.evaders):
            plt.plot(evader.position[0, 0], evader.position[1, 0], 'bo', label=f'Evader {i}')
        
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('Current Positions of Pursuers and Evaders')
        plt.legend()
        plt.grid()
        plt.show()