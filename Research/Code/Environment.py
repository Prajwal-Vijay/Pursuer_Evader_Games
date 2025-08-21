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
        return True

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