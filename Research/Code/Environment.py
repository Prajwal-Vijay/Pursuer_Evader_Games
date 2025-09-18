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
from scipy.optimize import minimize, Bounds
from matplotlib.patches import Patch

class Environment:
    def __init__(self, N, M, t, pursuers, evaders):
        self.N = N  # Number of pursuers
        self.M = M  # Number of evaders
        self.timestep = t  # Time step
        self.pursuers = pursuers  # Assuming a single pursuer for simplicity
        self.evaders = evaders
        self.active_evaders = evaders
        self.evader_str = dict()
        self.str_evader = dict()
        for evader in self.evaders:
            self.evader_str[evader] = str(evader)
            self.str_evader[str(evader)] = evader
        self.count = 0
        self.flow = None
        self.flow2 = None
        self.flow3 = None
    # WORKING FINE
    def check_initialization(self, verbose=False):
        """Check if the initial positions are valid for the simulation"""
        # Check if any evader is already captured or too close to the pursuer
        for pursuer in self.pursuers:
            for evader in self.active_evaders:
                if np.linalg.norm(pursuer.get_pos() - evader.get_pos()) < pursuer.capture_radius:
                    if verbose:
                        print(f"Evader {evader.index} is too close to the pursuer at initialization.")
                    return False
        for evader in self.active_evaders:
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
        for evader_idx, evader in enumerate(self.active_evaders):
            for coalition in coalition_list:
                if evasion_matrix[(evader, coalition)] == 1:
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
                min_z = self._compute_min_z_in_bes(evader, coalition)
                # Set matrix value: 1 if min_z > 0, -1 otherwise
                evasion_matrix[(evader, coalition)] = 1 if min_z > 0 else -1
        return evasion_matrix, coalition_list

    def _compute_min_z_in_bes(self, evader, coalition):
        # Uses the boundary of evasion space method as given in the paper.
        x0 = np.array([0,0,0])
        # FIXED BUG HERE: Minimization function is supposed to give the minima of their intersection, but it seems to be giving the minima of the last pursuers.
        # REASON : But Python’s lambda in a loop captures variables by reference, not by value. So by the time minimize uses those lambdas, all of them refer to the last pursuer_pos and evader_pos!
        def make_constraint(pursuer_pos, evader_pos, alpha_ij, capture_radius):
            return {
                'type': 'ineq',
                'fun': lambda x, p=pursuer_pos, e=evader_pos, a=alpha_ij, r=capture_radius:
                    np.linalg.norm(x - p) - a * np.linalg.norm(x - e) - r
            }
        bounds = Bounds([-100, -100, -np.inf],[+100, +100, +np.inf])
        constraints = []
        for pursuer_idx in coalition:
            pursuer = self.pursuers[pursuer_idx]
            evader_pos = np.array(evader.get_pos()).reshape(-1)
            pursuer_pos = np.array(pursuer.get_pos()).reshape(-1)
            alpha_ij = pursuer.speed/evader.speed
            capture_radius = pursuer.capture_radius
            constraints.append(make_constraint(pursuer_pos, evader_pos, alpha_ij, capture_radius))
        objective = lambda x: x[2]
        res = minimize(objective, x0, constraints=constraints, bounds=bounds, method='SLSQP')
        print(res.x)
        return res.x[2]
    
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
            ax.scatter(pursuer.position[0, 0], pursuer.position[1, 0], pursuer.position[2, 0], c='#FF0000', label=f'Pursuer {i}')
        for i, evader in enumerate(self.evaders):
            ax.scatter(evader.position[0, 0], evader.position[1, 0], evader.position[2, 0], c='#0000FF', label=f'Evader {i}')
        
        plt.legend()
        plt.grid()
        plt.show()
        plt.title("Pursuit Evasion Game")

    # DEBUG: The evader is not being caught, but it initialization is such that it can be caught, why is it failing?
    def update_termination(self):
        # Update the captured status of evaders
        for pursuer in self.pursuers:
            for evader in self.evaders:
                if np.linalg.norm(pursuer.get_pos() - evader.get_pos()) < pursuer.capture_radius:
                    evader.captured = True
    
    def step(self):
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
        self.active_evaders = [evader for i, evader in enumerate(self.evaders) if not evader.captured]
        if not self.active_evaders:
            return True
        # Calculate pursuer velocity
        win = self.check_initialization()
        # evader_positions = self.return_evader_positions(active_evaders)
        # From Here things get very different
        # YOUR REAL CODE STARTS HERE
        if not win:
            print("MATCH GONE")
        """
        Step 1: Compute the Valuefunction for each pair of pursuer coalition and the active evader.
        Step 2: Use the minCostMaxFlow Implementation to get the matching for 1v1 case.
        Step 3: Remove all coalitions that have pursuers that have been matched, and then use localSearchMaximum
        Step 4: do the same for 2v1 and 3v1
        Step 5: Find interception point for each of these cases.
        Step 6: Send the pursuers and evaders to the required positions
        """
        # STEP 1
        value_matrix, points_matrix, coalition_list = self.valueFunctionMatrix()
        print("VALUE MATRIX:",value_matrix)
        # STEP 2
        # Coalitions of size 1
        single_coalitions = [coalition for coalition in coalition_list if len(coalition) == 1]
        graph = dict()
        graph["So"] = dict()
        for single_coalition in single_coalitions:
            graph["So"][str(single_coalition)] = (1, 0)
        for single_coalition in single_coalitions:
            graph[str(single_coalition)] = dict()
            for evader in self.active_evaders:
                if value_matrix[(evader, single_coalition)] > 0:
                    graph[str(single_coalition)][self.evader_str[evader]] = (1, value_matrix[(evader, single_coalition)])
        for evader in self.active_evaders:
            graph[self.evader_str[evader]] = dict()
            graph[self.evader_str[evader]]["Si"] = (1, 0)
        graph["Si"] = dict()

        if self.count % 1000 == 0:
            self.flow = minCostMaxFlow_implemented.SuccessiveShortestPath(graph, "So", "Si")
        
        flow = self.flow
        print("Flow1:", flow)
        matched_pursuers = []
        matched_evaders = []
        # FIXED: HUGE ERROR HERE: See we need to make those combinations of pursuers and evaders move to their respective intercept points, whose flow value is 1 ONLY, the rest should remain still!!
        for (start, stop), value in flow.items():
            if value == 1:
                if start is not "So" and stop is not "Si" and start is not "Si" and start not in self.str_evader.keys():
                    start = eval(start)
                if stop in self.str_evader.keys():
                    stop = self.str_evader[stop]
                if type(start) is tuple and start[0] not in matched_pursuers:
                    matched_pursuers.append(int(start[0]))
                if stop not in matched_evaders:
                    matched_evaders.append(stop)
                if start in single_coalitions and stop in self.active_evaders:
                    for pursuer_idx in start:
                        pursuer_velocity = self.pursuers[pursuer_idx].heading_velocity(points_matrix[(stop, start)])
                        self.pursuers[pursuer_idx].update_pos(self.pursuers[pursuer_idx].position + self.timestep*pursuer_velocity)
                    if stop in self.active_evaders:
                        vel = stop.heading_velocity(points_matrix[(stop, start)])
                        stop.update_pos(stop.position+self.timestep*vel)

        # Coalitions of size 2 evaluated with swap of size 2
        dual_coalitions = [coalition for coalition in coalition_list if (len(coalition) == 2 and coalition[0] not in matched_pursuers and coalition[1] not in matched_pursuers)]
        

        # for dual_coalition in dual_coalitions:
        #     graph_2[dual_coalition] = dict()
        #     for evader in self.active_evaders:
        #         if value_matrix[(evader, dual_coalition)] != -1 and evader not in matched_evaders:
        #             graph_2[dual_coalition][evader] = (1, value_matrix[(evader, dual_coalition)])
        # BUG: Local_Search_Maximum is giving empty outputs
        # flow_2 = localSearchMaximum.localSearchMaximum(graph_2, set())
        
        # For now use the min_cost_max_flow implementation itself.
        graph_2 = dict()
        graph_2["So"] = dict()
        for dual_coalition in dual_coalitions:
            graph_2["So"][str(dual_coalition)] = (1, 0)
        for dual_coalition in dual_coalitions:
            graph_2[str(dual_coalition)] = dict()
            for evader in self.active_evaders:
                if value_matrix[(evader, dual_coalition)] > 0:
                    graph_2[str(dual_coalition)][self.evader_str[evader]] = (1, value_matrix[(evader, dual_coalition)])
        for evader in self.active_evaders:
            graph_2[self.evader_str[evader]] = dict()
            graph_2[self.evader_str[evader]]["Si"] = (1, 0)
        graph_2["Si"] = dict()
        if self.count % 1000 == 0:
            self.flow2 = minCostMaxFlow_implemented.SuccessiveShortestPath(graph_2, "So", "Si")
        flow_2 = self.flow2
        print("FLOW2:", flow_2)
        for (start, stop), value in flow_2.items():
            if value == 1:
                if start is not "So" and stop is not "Si" and start is not "Si" and start not in self.str_evader.keys():
                    start = eval(start)
                if stop in self.str_evader.keys():
                    stop = self.str_evader[stop]
                if type(start) is tuple and start[0] not in matched_pursuers:
                    matched_pursuers.append(int(start[0]))
                if type(start) is tuple and start[1] not in matched_pursuers:
                    matched_pursuers.append(int(start[1]))
                if stop not in matched_evaders:
                    matched_evaders.append(stop)
                if start in dual_coalitions and stop in self.active_evaders:
                    for pursuer_idx in start:
                        pursuer_velocity = self.pursuers[pursuer_idx].heading_velocity(points_matrix[(stop, start)])
                        self.pursuers[pursuer_idx].update_pos(self.pursuers[pursuer_idx].position + self.timestep*pursuer_velocity)
                    if stop in self.active_evaders:
                        vel = stop.heading_velocity(points_matrix[(stop, start)])
                        stop.update_pos(stop.position+self.timestep*vel)

        

        print("MATCHED PURSUERS:", matched_pursuers)
        triple_coalitions = [coalition for coalition in coalition_list if (len(coalition) == 3 and coalition[0] not in matched_pursuers and coalition[1] not in matched_pursuers and coalition[2] not in matched_pursuers)]
        graph_3 = dict()
        graph_3["So"] = dict()
        for triple_coalition in triple_coalitions:
            graph_3["So"][str(triple_coalition)] = (1, 0)
        for triple_coalition in triple_coalitions:
            graph_3[str(triple_coalition)] = dict()
            for evader in self.active_evaders:
                if value_matrix[(evader, triple_coalition)] > 0:
                    graph_3[str(triple_coalition)][self.evader_str[evader]] = (1, value_matrix[(evader, triple_coalition)])
        for evader in self.active_evaders:
            graph_3[self.evader_str[evader]] = dict()
            graph_3[self.evader_str[evader]]["Si"] = (1, 0)
        graph_3["Si"] = dict()
        if self.count % 1000 == 0:
            self.flow3 = minCostMaxFlow_implemented.SuccessiveShortestPath(graph_3, "So", "Si")
        flow_3 = self.flow3
        print("FLOW3:", flow_3)
        for (start, stop), value in flow_3.items():
            if value == 1:
                if start is not "So" and stop is not "Si" and start is not "Si" and start not in self.str_evader.keys():
                    start = eval(start)
                if stop in self.str_evader.keys():
                    stop = self.str_evader[stop]
                if type(start) is tuple and start[0] not in matched_pursuers:
                    matched_pursuers.append(int(start[0]))
                if type(start) is tuple and start[1] not in matched_pursuers:
                    matched_pursuers.append(int(start[1]))
                if type(start) is tuple and start[2] not in matched_pursuers:
                    matched_pursuers.append(int(start[2]))
                if stop not in matched_evaders:
                    matched_evaders.append(stop)
                if start in triple_coalitions and stop in self.active_evaders:
                    for pursuer_idx in start:
                        pursuer_velocity = self.pursuers[pursuer_idx].heading_velocity(points_matrix[(stop, start)])
                        self.pursuers[pursuer_idx].update_pos(self.pursuers[pursuer_idx].position + self.timestep*pursuer_velocity)
                    if stop in self.active_evaders:
                        vel = stop.heading_velocity(points_matrix[(stop, start)])
                        stop.update_pos(stop.position+self.timestep*vel)
        
        self.count += 1
        return False
    
    def valueFunctionMatrix(self, max_coalition_size=3):
        value_matrix = {}
        optimal_points_matrix = {}
        coalition_list = []
        for size in range(1, min(max_coalition_size + 1, len(self.pursuers) + 1)):
            for coalition in combinations(range(len(self.pursuers)), size):
                coalition_list.append(coalition) # coalition is not a list of pursuers, it is a list of numbers(indices)!!

        for evader_idx, evader in enumerate(self.active_evaders):
            for coalition in coalition_list:
                try:
                    value, optimal_point = self._solve_value_function_cvxpy(coalition, evader)
                    value_matrix[(evader, coalition)] = value
                    optimal_points_matrix[(evader, coalition)] = optimal_point
                except Exception as e:
                    print(f"Failed to compute value for coalition {coalition}, evader {evader}")
                    # value, point = self._fallback_value_function(coalition, evader)
                    value_matrix[(evader, coalition)] = -1
                    optimal_points_matrix[(evader, coalition)] = -1

        return value_matrix, optimal_points_matrix, coalition_list

    # def _solve_value_function_scipy(self, coalition, evader):
    #     evader_pos = np.array(evader.position)
    #     print(evader_pos.shape)
    #     init_pos = np.array([0, 0 ,0])

    def _solve_value_function_cvxpy(self, coalition, evader):
        # Uses the boundary of evasion space method as given in the paper.
        x0 = np.array([0,0,0])
        def make_constraint(pursuer_pos, evader_pos, alpha_ij, capture_radius):
            return {
                'type': 'ineq',
                'fun': lambda x, p=pursuer_pos, e=evader_pos, a=alpha_ij, r=capture_radius:
                    np.linalg.norm(x - p) - a * np.linalg.norm(x - e) - r
            }
        bounds = Bounds([-100, -100, -np.inf], [+100, +100, +np.inf])
        constraints = []
        for pursuer_idx in coalition:
            pursuer = self.pursuers[pursuer_idx]
            evader_pos = np.array(evader.get_pos()).reshape(-1)
            pursuer_pos = np.array(pursuer.get_pos()).reshape(-1)
            alpha_ij = pursuer.speed/evader.speed
            capture_radius = pursuer.capture_radius
            constraints.append(make_constraint(pursuer_pos, evader_pos, alpha_ij, capture_radius))
        objective = lambda x: x[2]
        res = minimize(objective, x0, constraints=constraints, bounds=bounds, method='SLSQP')
        return res.x[2], res.x # Returns the minimum distance as well as the point

    def obtain_trajectories(self, max_steps=10000, animation_speed=0.1):
        """
        Run the simulation and display real-time 3D animation with trajectory curves
        
        Args:
            max_steps (int): Maximum number of simulation steps
            animation_speed (float): Delay between frames in seconds
        """
        plt.ion()  # Turn on interactive mode
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Initialize trajectory storage
        pursuer_trajectories = [[] for _ in range(len(self.pursuers))]
        evader_trajectories = [[] for _ in range(len(self.evaders))]
        
        # Store initial positions
        for i, pursuer in enumerate(self.pursuers):
            pursuer_trajectories[i].append(pursuer.get_pos().flatten())
        for i, evader in enumerate(self.evaders):
            evader_trajectories[i].append(evader.get_pos().flatten())

        # Colors for different agents
        pursuer_colors = plt.cm.Reds(np.linspace(0.4, 1, len(self.pursuers)))
        evader_colors = plt.cm.Blues(np.linspace(0.4, 1, len(self.evaders)))

        step_count = 0
        game_over = False
        
        print("Starting pursuit-evasion simulation...")
        print("Red = Pursuers, Blue = Evaders")
        print("Goal: Evaders try to reach z=0 plane, Pursuers try to capture them")
        
        try:
            while step_count < max_steps and not game_over:
                print(f"=== Starting step {step_count} ===")
                
                try:
                    # Clear the plot
                    print("Clearing plot...")
                    ax.clear()
                    
                    # Set up the 3D environment
                    print("Setting up 3D environment...")
                    ax.set_xlabel('X Position')
                    ax.set_ylabel('Y Position')
                    ax.set_zlabel('Z Position')
                    ax.set_title(f'Pursuit-Evasion Game - Step {step_count}')

                    # Set reasonable axis limits based on current positions
                    print("Calculating axis limits...")
                    all_positions = []  # Reset the list each iteration
                    
                    for pursuer in self.pursuers:
                        pos = pursuer.get_pos().flatten()
                        all_positions.append(pos)
                        print(f"Pursuer position: {pos}")
                        
                    for evader in self.evaders:
                        if not evader.captured:
                            pos = evader.get_pos().flatten()
                            all_positions.append(pos)
                            print(f"Evader position: {pos}")

                    if all_positions:
                        all_positions = np.array(all_positions)
                        print(f"All positions shape: {all_positions.shape}")
                        margin = 5
                        ax.set_xlim(np.min(all_positions[:, 0]) - margin, np.max(all_positions[:, 0]) + margin)
                        ax.set_ylim(np.min(all_positions[:, 1]) - margin, np.max(all_positions[:, 1]) + margin)
                        ax.set_zlim(np.min(all_positions[:, 2]) - margin, np.max(all_positions[:, 2]) + margin)
                        print("Set dynamic axis limits")
                    else:
                        ax.set_xlim(-20, 20)
                        ax.set_ylim(-20, 20)
                        ax.set_zlim(-20, 20)
                        print("Set default axis limits")
                    
                    # Draw the goal plane (z = 0)
                    print("Drawing goal plane...")
                    try:
                        xx, yy = np.meshgrid(np.linspace(ax.get_xlim()[0], ax.get_xlim()[1], 10),
                                        np.linspace(ax.get_ylim()[0], ax.get_ylim()[1], 10))
                        zz = np.zeros_like(xx)
                        ax.plot_surface(xx, yy, zz, alpha=0.2, color='green')
                        print("Goal plane drawn successfully")
                    except Exception as e:
                        print(f"Error drawing goal plane: {e}")
                        # Continue without the goal plane
                    
                    # Plot pursuer trajectories and current positions
                    print("Plotting pursuers...")
                    for i, pursuer in enumerate(self.pursuers):
                        try:
                            if len(pursuer_trajectories[i]) > 1:
                                traj = np.array(pursuer_trajectories[i])
                                ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                                    color=pursuer_colors[i], alpha=0.7, linewidth=2)
                            
                            # Current position
                            pos = pursuer.get_pos().flatten()
                            ax.scatter(pos[0], pos[1], pos[2], 
                                    color=pursuer_colors[i], s=100, marker='o',
                                    edgecolors='black', linewidth=2)
                            
                            # Skip capture radius visualization for now to isolate the issue
                            # We'll add it back once we confirm the basic plotting works
                            
                        except Exception as e:
                            print(f"Error plotting pursuer {i}: {e}")
                            continue
                    
                    # Plot evader trajectories and current positions
                    print("Plotting evaders...")
                    for i, evader in enumerate(self.evaders):
                        try:
                            if not evader.captured:
                                if len(evader_trajectories[i]) > 1:
                                    traj = np.array(evader_trajectories[i])
                                    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                                        color=evader_colors[i], alpha=0.7, linewidth=2)
                                
                                # Current position
                                pos = evader.get_pos().flatten()
                                ax.scatter(pos[0], pos[1], pos[2], 
                                        color=evader_colors[i], s=100, marker='^',
                                        edgecolors='black', linewidth=2)
                            else:
                                # Show captured evaders as faded
                                if len(evader_trajectories[i]) > 1:
                                    traj = np.array(evader_trajectories[i])
                                    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                                        color='gray', alpha=0.3, linewidth=1,
                                        linestyle='--')
                                
                                pos = evader.get_pos().flatten()
                                ax.scatter(pos[0], pos[1], pos[2], 
                                        color='gray', s=50, marker='x',
                                        alpha=0.5)
                        except Exception as e:
                            print(f"Error plotting evader {i}: {e}")
                            continue
                    
                    # Add status text
                    print("Adding status text...")
                    try:
                        active_evaders_count = len([e for e in self.evaders if not e.captured])
                        captured_evaders_count = len(self.evaders) - active_evaders_count
                        
                        status_text = f"Step: {step_count}\n"
                        status_text += f"Active Evaders: {active_evaders_count}\n"
                        status_text += f"Captured Evaders: {captured_evaders_count}"
                        
                        ax.text2D(0.02, 0.98, status_text, transform=ax.transAxes, 
                                fontsize=10, verticalalignment='top',
                                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
                        print("Status text added")
                    except Exception as e:
                        print(f"Error adding status text: {e}")
                    
                    # Draw the plot
                    print("Drawing plot...")
                    plt.draw()
                    plt.pause(animation_speed)
                    print("Plot drawn successfully")
                    
                except Exception as e:
                    print(f"Error in plotting section of step {step_count}: {e}")
                    print("Full traceback:")
                    traceback.print_exc()
                    break
                
                # Execute one simulation step
                print("Executing simulation step...")
                try:
                    game_over = self.step()
                    print(f"Step {step_count} completed: game_over = {game_over}")
                except Exception as e:
                    print(f"Error in self.step(): {e}")
                    traceback.print_exc()
                    break
                
                # Store new positions for trajectory plotting
                print("Storing new positions...")
                try:
                    for i, pursuer in enumerate(self.pursuers):
                        pursuer_trajectories[i].append(pursuer.get_pos().flatten())
                    for i, evader in enumerate(self.evaders):
                        evader_trajectories[i].append(evader.get_pos().flatten())
                    print("Positions stored successfully")
                except Exception as e:
                    print(f"Error storing positions: {e}")
                    break
                
                step_count += 1
                print(f"=== Completed step {step_count - 1} ===\n")
                # Check termination conditions
                if game_over:
                    print(f"\nGame ended at step {step_count}")
                    
                    # Determine the outcome
                    escaped_evaders = []
                    captured_evaders = []
                    
                    for i, evader in enumerate(self.evaders):
                        if evader.captured:
                            captured_evaders.append(i)
                        elif evader.get_pos()[2, 0] <= 0:  # Reached goal
                            escaped_evaders.append(i)
                    
                    if escaped_evaders:
                        print(f"Evaders {escaped_evaders} successfully escaped!")
                    if captured_evaders:
                        print(f"Evaders {captured_evaders} were captured!")
                    
                    break

        except KeyboardInterrupt:
            print(f"\nSimulation interrupted at step {step_count}")

        finally:
            # Final plot with complete trajectories
            ax.clear()
            ax.set_xlabel('X Position')
            ax.set_ylabel('Y Position')
            ax.set_zlabel('Z Position')
            ax.set_title(f'Final Trajectories - Pursuit-Evasion Game')

            # Plot goal plane
            if all_positions:
                xx, yy = np.meshgrid(np.linspace(ax.get_xlim()[0], ax.get_xlim()[1], 10),
                                np.linspace(ax.get_ylim()[0], ax.get_ylim()[1], 10))
                zz = np.zeros_like(xx)
                ax.plot_surface(xx, yy, zz, alpha=0.2, color='green')
            
            # Plot final trajectories
            for i, traj in enumerate(pursuer_trajectories):
                if len(traj) > 1:
                    traj = np.array(traj)
                    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                        color=pursuer_colors[i], linewidth=3, alpha=0.8,
                        label=f'Pursuer {i}')
                    # Mark start and end
                    ax.scatter(traj[0, 0], traj[0, 1], traj[0, 2], 
                            color=pursuer_colors[i], s=150, marker='s', 
                            edgecolors='black', linewidth=2)
                    ax.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2], 
                            color=pursuer_colors[i], s=150, marker='o',
                            edgecolors='black', linewidth=2)
            
            for i, traj in enumerate(evader_trajectories):
                if len(traj) > 1:
                    traj = np.array(traj)
                    color = 'gray' if self.evaders[i].captured else evader_colors[i]
                    alpha = 0.4 if self.evaders[i].captured else 0.8
                    linestyle = '--' if self.evaders[i].captured else '-'
                    
                    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], 
                        color=color, linewidth=3, alpha=alpha, linestyle=linestyle,
                        label=f'Evader {i}' + (' (captured)' if self.evaders[i].captured else ''))
                    # Mark start and end
                    ax.scatter(traj[0, 0], traj[0, 1], traj[0, 2], 
                            color=color, s=150, marker='D', 
                            edgecolors='black', linewidth=2, alpha=alpha)
                    ax.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2], 
                            color=color, s=150, marker='^',
                            edgecolors='black', linewidth=2, alpha=alpha)
            
            ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
            
            # Add final statistics
            final_stats = f"Simulation completed in {step_count} steps\n"
            final_stats += f"Captured: {len([e for e in self.evaders if e.captured])}\n"
            final_stats += f"Escaped: {len([e for e in self.evaders if not e.captured and e.get_pos()[2,0] <= 0])}\n"
            final_stats += f"Still active: {len([e for e in self.evaders if not e.captured and e.get_pos()[2,0] > 0])}"
            
            ax.text2D(0.02, 0.02, final_stats, transform=ax.transAxes, 
                    fontsize=10, verticalalignment='bottom',
                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
            
            plt.ioff()  # Turn off interactive mode
            plt.show()
            
            return {
                'steps': step_count,
                'pursuer_trajectories': pursuer_trajectories,
                'evader_trajectories': evader_trajectories,
                'final_positions': {
                    'pursuers': [p.get_pos().flatten() for p in self.pursuers],
                    'evaders': [e.get_pos().flatten() for e in self.evaders]
                },
                'captured_evaders': [i for i, e in enumerate(self.evaders) if e.captured],
                'escaped_evaders': [i for i, e in enumerate(self.evaders) 
                                if not e.captured and e.get_pos()[2,0] <= 0]
            }
        