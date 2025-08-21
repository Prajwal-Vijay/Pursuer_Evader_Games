import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize_scalar
import time
import Evader
import Pursuer

class Environment:
    def __init__(self, N, n, v, u, r, pursuer_position=None, evader_positions=None, target_position=None):
        self.motion_space_dimension = N
        self.evader_numbers = n
        self.pursuer_speed = v
        self.evader_speeds = u
        self.timestep = r
        self.captured_evaders = np.zeros(n, dtype=bool)
        self.capture_tolerance = 0.05
        self.alpha = np.array(u)/v

        # Initialize pursuer
        if pursuer_position is not None:
            self.pursuer = Pursuer.Pursuer(pursuer_position, v)
        else:
            self.pursuer = Pursuer.Pursuer(np.random.rand(N, 1), v)
        
        # Initialize evaders
        self.evaders = []
        if evader_positions is not None:
            for i in range(n):
                self.evaders.append(Evader.Evader(evader_positions[:, i:i+1], u[i], i))
        else:
            for i in range(n):
                self.evaders.append(Evader.Evader(np.random.rand(N, 1), u[i], i))
        
        # Initialize target
        if target_position is not None:
            self.target_position = np.array(target_position).reshape(-1, 1)
        else:
            self.target_position = np.random.rand(N, 1)
    
    def update_target(self, target_position):
        target_position = np.array(target_position)
        if target_position.shape != self.target_position.shape:
            if target_position.T.shape == self.target_position.shape:
                self.target_position = target_position.T
            else:
                raise ValueError("Wrong position dimensions")
        else:
            self.target_position = target_position
    
    def barrier(self, evaders):
        barriers = np.zeros(len(evaders))
        for i, evader in enumerate(evaders):
            barriers[i] = (np.linalg.norm(self.target_position-evader.position)**2-
                           self.alpha[evader.index]**2 * np.linalg.norm(self.target_position-self.pursuer.position)**2)
        return np.min(barriers)
    
    def check_initialization(self, evaders, display_info=True):
        if self.barrier(evaders) < 0:
            win = False
        else:
            win = True
        return win

    def plot_current_positions(self):
        plt.figure(figsize=(10, 8))
        for evader in self.evaders:
            plt.plot(evader.position[0,0], evader.position[1,0], 'bo', markersize=10, label='Evader' if evader.index == 0 else "")
        # Plot target
        plt.plot(self.target_position[0,0], self.target_position[1,0], 'go', markersize=10, label='Target')
        # Plot pursuer
        plt.plot(self.pursuer.position[0,0], self.pursuer.position[1,0], 'ro', markersize=10, label='Pursuer')

        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.title('Pursuit-Evasion Game')

    def return_evader_positions(self, evaders):
        positions = np.zeros((self.motion_space_dimension, len(evaders)))
        for i, evader in enumerate(evaders):
            positions[:, i] = evader.position.flatten()
        return positions
    
    def return_evader_velocities(self, evaders):
        velocities = np.zeros((self.motion_space_dimension, len(evaders)))
        for i, evader in enumerate(self.evaders):
            # When evader has been captured, its velocity is 0
            if not self.captured_evaders[i]:
                win = self.check_initialization([evader], False)
                # Uses the position of pursuer, target, and ratio of speed to find the velocity.
                velocity, _ = evader.heading_velocity(self.pursuer.position, self.target_position, win, self.alpha[i])
                velocities[:, i] = velocity.flatten()
        return velocities

    def update_termination(self):
        # Update the captured status of evaders
        for i, evader in enumerate(self.evaders):
            if np.linalg.norm(self.pursuer.position - evader.position) < self.capture_tolerance:
                self.captured_evaders[i] = True
    
    def step(self, objective_function):
        # Executes one step of the simulation at a time
        self.update_termination()
        # Check if all the evaders have been captured
        if np.all(self.captured_evaders):
            return True
        # Check if any evader has reached the target
        for i, evader in enumerate(self.evaders):
            if not self.captured_evaders[i]:
                if (np.linalg.norm(evader.position - self.target_position) < self.capture_tolerance and 
                    np.linalg.norm(self.pursuer.position - self.target_position) > 2*self.capture_tolerance):
                    print(f'{evader.name} reached the target')
                    return True
        # All active evaders are done, stop game
        active_evaders = [evader for i, evader in enumerate(self.evaders) if not self.captured_evaders[i]]
        if not active_evaders:
            return True
        # Calculate pursuer velocity
        win = self.check_initialization(active_evaders, False)
        evader_positions = self.return_evader_positions(active_evaders)
        # This is the complex mathematics function that calculates pursuer velocity
        pursuer_velocity, _ = self.pursuer.heading_velocity(evader_positions, self.target_position, self.timestep, win, objective_function)
        evader_velocities = self.return_evader_velocities(self.evaders)
        # Update positions
        self.pursuer.update_pos(self.pursuer.position + self.timestep*pursuer_velocity)
        for i, evader in enumerate(self.evaders):
            if not self.captured_evaders[i]:
                evader.update_pos(evader.position+self.timestep * evader_velocities[:,i:i+1])
        return False
#    def obtain_trajectories(self, objective_function, max_steps=10000):
#        self.plot_current_positions()
#        plt.show()
#        done = False
#        t = 0
#        pursuer_positions_traj = []
#        evader_positions_traj = [[] for _ in range(self.evader_numbers)]
#        while not done and t<max_steps:
#            pursuer_positions_traj.append(self.pursuer.position.copy())
#            for i, evader in enumerate(self.evaders):
#                evader_positions_traj[i].append(evader.position.copy())
#            done = self.step(objective_function)
#            t += 1
#            self.plot_trajectories(pursuer_positions_traj, evader_positions_traj)
#            if t % 100 == 0:
#                print(f"Step {t}")
#        
#        # Store final positions
#        pursuer_positions_traj.append(self.pursuer.position.copy())
#        for i, evader in enumerate(self.evaders):
#            evader_positions_traj[i].append(evader.position.copy())
#        # Determine winner
#        win = np.all(self.captured_evaders)
#        # Plot final trajectories
#        self.plot_trajectories(pursuer_positions_traj, evader_positions_traj)
#
#        return win, pursuer_positions_traj, evader_positions_traj

    def obtain_trajectories(self, objective_function, max_steps=10000):
        # Setup interactive plotting
        plt.ion()
        fig, ax = plt.subplots(figsize=(10, 8))

        # Initial plot elements
        pursuer_plot, = ax.plot([], [], 'ro', markersize=10, label='Pursuer')
        evader_plots = [ax.plot([], [], 'bo', markersize=10, label='Evader' if i == 0 else "")[0] for i in range(self.evader_numbers)]
        target_plot, = ax.plot(self.target_position[0, 0], self.target_position[1, 0], 'go', markersize=10, label='Target')
        pursuer_traj_line, = ax.plot([], [], 'r-', linewidth=2, label='Pursuer Trajectory')
        evader_traj_lines = [ax.plot([], [], 'b-', linewidth=2, label=f'Evader {i} Trajectory' if i == 0 else "")[0] for i in range(self.evader_numbers)]

        ax.legend()
        ax.grid(True)
        ax.set_aspect('equal')
        ax.set_title("Pursuit-Evasion Game Animation")

        pursuer_positions_traj = []
        evader_positions_traj = [[] for _ in range(self.evader_numbers)]

        done = False
        t = 0
        while not done and t < max_steps:
            pursuer_positions_traj.append(self.pursuer.position.copy())
            for i, evader in enumerate(self.evaders):
                evader_positions_traj[i].append(evader.position.copy())

            # Update positions
            done = self.step(objective_function)

            # Update plot
            pursuer_plot.set_data([self.pursuer.position[0, 0]], [self.pursuer.position[1, 0]])
            for i, evader in enumerate(self.evaders):
                if not self.captured_evaders[i]:
                    evader_plots[i].set_data([evader.position[0, 0]], [evader.position[1, 0]])

            # Update trajectory lines
            pursuer_x = [p[0, 0] for p in pursuer_positions_traj]
            pursuer_y = [p[1, 0] for p in pursuer_positions_traj]
            pursuer_traj_line.set_data(pursuer_x, pursuer_y)

            for i, traj in enumerate(evader_positions_traj):
                evader_x = [p[0, 0] for p in traj]
                evader_y = [p[1, 0] for p in traj]
                evader_traj_lines[i].set_data(evader_x, evader_y)

            ax.relim()
            ax.autoscale_view()

            plt.pause(0.00000000000001)
            t += 1
            if t % 100 == 0:
                print(f"Step {t}")

        plt.ioff()  # Turn off interactive mode after done
        plt.show()

        # Determine winner
        win = np.all(self.captured_evaders)
        return win, pursuer_positions_traj, evader_positions_traj

    """ NOT BEING USED
    def plot_trajectories(self, pursuer_positions_traj, evader_positions_traj):
        plt.figure(figsize=(12, 10))
        
        # Plot pursuer trajectory
        pursuer_x = [pos[0, 0] for pos in pursuer_positions_traj]
        pursuer_y = [pos[1, 0] for pos in pursuer_positions_traj]
        plt.plot(pursuer_x, pursuer_y, 'r-', label='Pursuer trajectory', linewidth=2)
        
        # Plot evader trajectories
        for i, traj in enumerate(evader_positions_traj):
            evader_x = [pos[0, 0] for pos in traj]
            evader_y = [pos[1, 0] for pos in traj]
            plt.plot(evader_x, evader_y, 'b-', label=f'Evader {i} trajectory' if i == 0 else "", linewidth=2)
        
        # Plot current positions
        for evader in self.evaders:
            plt.plot(evader.position[0, 0], evader.position[1, 0], 'bo', markersize=10)
        
        plt.plot(self.target_position[0, 0], self.target_position[1, 0], 'go', markersize=10, label='Target')
        plt.plot(self.pursuer.position[0, 0], self.pursuer.position[1, 0], 'ro', markersize=10, label='Pursuer')
        
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        plt.title('Pursuit-Evasion Game Trajectories')
        plt.show()
    """