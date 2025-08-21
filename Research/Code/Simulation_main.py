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
    N=1 # Number of pursuers
    M=1 # Number of evaders



    pursuer_position = np.array([[-1.7720],[0.3751]])
    evader_positions = np.array([[9.1501, -6.8477], [9.2978, 9.4119]])
    target_position = np.array([[0], [0]])
    env = Environment.Environment(N, n, v, u, r, pursuer_position, evader_positions, target_position)
    win = env.check_initialization(env.evaders, True)
    env.plot_current_positions()
    # We are trying to avoid the condition B_ij >= 0 and alpha_ij >= 1, if this happens
    # we will not even run the simulation, because the evaders will endup winning irrespective of how
    # smartly the pursuers play.
    if win:
        print("Running simulation")
        win_result, pursuer_traj, evader_traj = env.obtain_trajectories('heuristic')
        if win_result:
            print('Pursuer wins!')
        else:
            print('Evaders win!')
    else:
        print('Irrespecitive of how the pursuer plays, evaders end up winning. UNFAIR!')

if __name__ == "__main__":
    main()
