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
from mpl_toolkits import mplot3d
import minCostMaxFlow_implemented
import localSearchMaximum

class Environment:
    def __init__(self, N, M, t, pursuers, evaders):
        self.N = N  # Number of pursuers
        self.M = M  # Number of evaders
        self.timestep = t  # Time step
        self.pursuers = pursuers  # Assuming a single pursuer for simplicity
        self.evaders = evaders
        

    def check_initialization(self, verbose=False, evaders=None):
        if not evaders:
            evaders = self.evaders
        """Check if the initial positions are valid for the simulation"""
        # Check if any evader is already captured or too close to the pursuer
        for pursuer in self.pursuers:
            for evader in evaders:
                if np.linalg.norm(pursuer.get_pos() - evader.get_pos()) < pursuer.capture_radius:
                    if verbose:
                        print(f"Evader {evader.index} is too close to the pursuer at initialization.")
                    return False
        for evader in evaders:
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
        for evader_idx, evader in enumerate(evaders):
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
        plt.figure()
        ax = plt.axes(projection="3d")
        # Data for a three-dimensional line
        zline = np.linspace(-15, 15, 1000)
        xline = np.linspace(-15, 15, 1000)
        yline = np.linspace(-15, 15, 1000)
        ax.plot3D(xline, yline, zline, 'gray')

        for i, pursuer in enumerate(self.pursuers):
            ax.scatter(pursuer.position[0, 0], pursuer.position[1, 0], pursuer.position[2, 0], c='ro', marker='Pursuer {i}')
        for i, evader in enumerate(self.evaders):
            plt.plot(evader.position[0, 0], evader.position[1, 0], evader.position[3, 0], c='bo', marker=f'Evader {i}')
        
        plt.legend()
        plt.grid()
        plt.show()
        plt.title("Pursuit Evasion Game")

    
    def update_termination(self):
        # Update the captured status of evaders
        for pursuer in self.pursuers:
            for evader in self.evaders:
                if np.linalg.norm(pursuer.get_pos() - evader.get_pos()) < pursuer.capture_radius:
                    evader.captured = True
    

    def step(self, objective_function):
        # Executes one step of the simulation at a time
        self.update_termination()
        
        # Check if all the evaders have been captured
        for evader in self.evaders:
            if not evader.captured:
                break
        else:
            return True
        
        # Check if any evader has reached the target
        for i, evader in enumerate(self.evaders):
            if not evader.captured:
                if (evader.position[2, 0] < 0):
                    print(f'{evader.name} reached the target')
                    return True
                
        # All active evaders are done, stop game
        active_evaders = [evader for i, evader in enumerate(self.evaders) if not evader.captured]
        if not active_evaders:
            return True
        # Calculate pursuer velocity
        win = self.check_initialization(False, active_evaders)
        # evader_positions = self.return_evader_positions(active_evaders)
        # From Here things get very different
        # YOUR REAL CODE STARTS HERE
        """
        Step 1: Compute the Valuefunction for each pair of pursuer coalition and the active evader.
        Step 2: Use the minCostMaxFlow Implementation to get the matching for 1v1 case.
        Step 3: Remove all coalitions that have pursuers that have been matched, and then use localSearchMaximum
        Step 4: do the same for 2v1 and 3v1
        Step 5: Find interception point for each of these cases.
        Step 6: Send the pursuers and evaders to the required positions
        """
        

        pursuer_velocity, _ = self.pursuer.heading_velocity(evader_positions, self.target_position, self.timestep, win, objective_function)
        evader_velocities = self.return_evader_velocities(self.evaders)

        # Update positions
        self.pursuer.update_pos(self.pursuer.position + self.timestep*pursuer_velocity)
        for i, evader in enumerate(self.evaders):
            if not self.captured_evaders[i]:
                evader.update_pos(evader.position+self.timestep * evader_velocities[:,i:i+1])
        return False
