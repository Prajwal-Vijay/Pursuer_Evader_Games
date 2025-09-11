"""
This includes the main initialization of the states of evaders and pursuers.
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize_scalar
import time
import Environment
import Pursuer
import Evader
import random

def main():
    plt.close('all')
    N=2 # Number of pursuers
    M=1 # Number of evaders
    t = 0.01 # Time step
    pursuer_positions = np.array([[0, 0.3751, 0.8761]])
    evader_positions = np.array([[-1.7720, 0.3751, 9.2978]])
    pursuer_speeds = np.array([1.5, 1])
    evader_speeds = np.array([1])
    pursuers = []
    evaders = []

    for i in range(M):
        evaders.append(Evader.Evader(evader_positions[i], evader_speeds[i], i))
    for i in range(N):
        pursuers.append(Pursuer.Pursuer(pursuer_positions[i], pursuer_speeds[i], i))
    env = Environment.Environment(N, M, t, pursuers, evaders)
    win = env.check_initialization(True)
    print(win)
    env.plot_current_positions()
    # We are trying to avoid the condition B_ij >= 0 and alpha_ij >= 1, if this happens
    # we will not even run the simulation, because the evaders will endup winning irrespective of how
    # smartly the pursuers play.
    
    if win:
        print("Running simulation")
        win_result = env.obtain_trajectories()
        print(win_result)
    else:
        print('Irrespecitive of how the pursuer plays, evaders end up winning. UNFAIR!')

if __name__ == "__main__":
    main()
