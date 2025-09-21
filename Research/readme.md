# Weighted Matching-Based Capture Strategies for 3D Heterogeneous Multi-Player Reach-Avoid Differential Games
<img width="1024" height="1024" alt="image (1)" src="https://github.com/user-attachments/assets/b1f5382d-5aaf-405f-8024-b0dbfbe6d81e" />

## Overview

This repository implements a novel approach to solving **3D heterogeneous multi-player reach-avoid differential games** using **matching-based capture strategies** with **weighted value functions**. The implementation extends the traditional matching algorithms by incorporating value functions as weights, providing an improvement over existing literature in the field of pursuit-evasion games.

This repository also presents an exhaustive python simulation framework for pursuer evader differential games.

## Table of Contents

1. [Theoretical Background](#theoretical-background)
2. [Problem Formulation](#problem-formulation)
3. [Key Innovations](#key-innovations)
4. [Implementation Architecture](#implementation-architecture)
5. [Mathematical Framework](#mathematical-framework)
6. [Algorithm Overview](#algorithm-overview)
7. [Simulation Features](#simulation-features)
8. [Demo Section](#demo-section)
9. [Usage](#usage)
10. [Research Applications](#research-applications)
11. [References](#references)

## Theoretical Background

### Differential Games Theory

**Differential games** are dynamic games played in continuous time where the evolution of the system state is governed by differential equations. In our context, we consider a **zero-sum differential game** between two adversarial teams:

- **Pursuers (Defenders)**: Aim to capture evaders and protect target regions
- **Evaders (Attackers)**: Attempt to reach designated target regions while avoiding capture

### Hamilton-Jacobi-Isaacs (HJI) Equations

The theoretical foundation of our approach lies in solving the **Hamilton-Jacobi-Isaacs equation**, which characterizes the value function of the differential game:

```
∂V/∂t + H(x, t, ∇V) = 0
```

Where:
- `V(x,t)` is the value function
- `H` is the Hamiltonian defined as `H = max_{u∈U} min_{v∈V} ⟨∇V, f(x,u,v)⟩`
- `f(x,u,v)` represents the system dynamics

The **value function** represents the minimum distance an evader can reach to the goal before being captured by the pursuers, serving as a crucial metric for optimal strategy computation.

### Reach-Avoid Games

In **reach-avoid differential games**, evaders have dual objectives:
1. **Reach**: Arrive at a predefined target region
2. **Avoid**: Evade capture by pursuers

This creates a more complex strategic environment compared to simple pursuit-evasion games, as players must balance offensive and defensive strategies simultaneously.

## Problem Formulation

### System Dynamics

Consider a 3D heterogeneous multi-player system with:
- `N` pursuers with positions `p_i ∈ ℝ³` and speeds `v_p,i`
- `M` evaders with positions `e_j ∈ ℝ³` and speeds `v_e,j`
- Target region `T ⊂ ℝ³` (typically defined as `z ≤ 0` plane)
- Capture radius `r_c` for each pursuer

### Game Objectives

**Evaders**: 
The Evasion team tries to send as many evaders as possible into the Target region, or get as close as possible to it.

**Pursuers**:
The pursuit team aims to caputure as many evaders as possible before they enter the goal region, while trying to maximize the distance at which the evaders are caught.

## Key Innovations

### 1. Weighted Value Function Matching

Our primary contribution extends traditional bipartite matching by using **value functions as edge weights**, where the weight `w_{ij}` between pursuer coalition `i` and evader `j` represents the optimal interception value.

### 2. Coalition-Based Pursuit

The algorithm considers pursuer coalitions of varying sizes:
- **1v1**: Single pursuer vs single evader
- **2v1**: Two-pursuer coalition vs single evader  
- **3v1**: Three-pursuer coalition vs single evader
![Uploading image.png…]()

### 3. Boundary of Evasion Space (BES)

We compute optimal interception points using the **Boundary of Evasion Space** method:

```
BES = {x ∈ ℝ³ : ||x - p_i|| - α_{ij}||x - e_j|| = r_c}
```

Where `α_{ij} = v_p,i / v_e,j` is the speed ratio.

## Implementation Architecture

### Core Components

1. **Environment.py**: Main simulation environment containing:
   - Value function computation using HJI equations
   - Matching algorithm implementation
   - Coalition formation logic
   - 3D visualization and trajectory tracking

2. **Agent Classes**:
   - **Pursuer.py**: Individual pursuer behavior and dynamics
   - **Evader.py**: Individual evader behavior and dynamics
   - **Coalition.py**: Multi-pursuer coordination

3. **Optimization Algorithms**:
   - **minCostMaxFlow_implemented.py**: Min-cost max-flow for optimal matching
   - **localSearchMaximum.py**: Local search optimization for larger coalitions

4. **Simulation_main.py**: Main execution script with initialization

### System Architecture Diagram

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Environment   │◄──►│  Value Function  │◄──►│   HJI Solver    │
│                 │    │   Computation    │    │                 │
└─────────┬───────┘    └──────────────────┘    └─────────────────┘
          │
          ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Coalition       │◄──►│ Matching         │◄──►│ Min-Cost        │
│ Formation       │    │ Algorithm        │    │ Max-Flow        │
└─────────────────┘    └──────────────────┘    └─────────────────┘
          │
          ▼
┌─────────────────┐    ┌──────────────────┐
│ Agents          │◄──►│ 3D Simulation    │
│ (P & E)         │    │ & Visualization  │
└─────────────────┘    └──────────────────┘
```

## Mathematical Framework

### Value Function Computation

The value function `V(x,t)` is computed by solving the optimization problem:

```
V(x,t) = min_{x∈BES} x_z
```

Subject to the constraints:
```
||x - p_i|| - α_{ij}||x - e_j|| ≥ r_c  ∀i ∈ coalition
```

### Matching Problem Formulation

Given:
- Set of pursuer coalitions `C = {C_1, C_2, ..., C_k}`
- Set of active evaders `E = {e_1, e_2, ..., e_m}`
- Value function weights `w(C_i, e_j)`

Find the maximum cardinality matching, and among those matchings which have the maximum cardinality, try to maximize the sum of weights:
```
max ∑_{(C_i,e_j)∈M} w(C_i, e_j)
```

Subject to:
- Each evader matched to at most one coalition
- Each pursuer appears in at most one active coalition

### Bipartite Graph Construction

The matching problem is formulated as a **weighted bipartite graph** `G = (U ∪ V, E)` where:
- `U`: Set of pursuer coalitions
- `V`: Set of evaders
- Edge weights: `w(u,v) = V(e_v, C_u)` (value function)

## Algorithm Overview

### Main Algorithm Steps

```python
def solve_game_step():
    # Step 1: Compute value function matrix
    value_matrix, points_matrix = compute_value_functions()
    # Since finding the entire optimal matching is an NP Hard problem, we need to solve it this approximation.
    # Step 2: Solve 1v1 matching using min-cost max-flow
    matching_1v1 = min_cost_max_flow(single_coalitions, evaders, value_matrix)
    
    # Step 3: Solve 2v1 matching for remaining agents
    matching_2v1 = min_cost_max_flow(dual_coalitions, remaining_evaders, value_matrix)
    
    # Step 4: Solve 3v1 matching for remaining agents
    matching_3v1 = min_cost_max_flow(triple_coalitions, remaining_evaders, value_matrix)
    
    # Step 5: Execute optimal strategies
    execute_strategies(matchings, points_matrix)
```

### Value Function Optimization

```python
def compute_value_function(coalition, evader):
    # Define optimization problem
    def objective(x):
        return x[2]  # Minimize z-coordinate
    
    # Boundary of Evasion Space constraints
    constraints = []
    for pursuer in coalition:
        constraint = lambda x: (
            np.linalg.norm(x - pursuer.pos) - 
            alpha * np.linalg.norm(x - evader.pos) - 
            capture_radius
        )
        constraints.append({'type': 'ineq', 'fun': constraint})
    
    # Solve optimization
    result = minimize(objective, x0, constraints=constraints)
    return result.fun, result.x
```

## Simulation Features

### 3D Visualization
- Real-time trajectory plotting for all agents
- Target region visualization (z = 0 plane)
- Capture radius indicators
- Dynamic camera positioning

### Performance Metrics
- Capture success rate
- Average capture time
- Trajectory optimality measures
  
### Configurable Parameters
- Number of pursuers and evaders
- Agent speeds and capabilities
- Target region definition
- Capture radius settings
- Time step resolution

## Demo Section


### Custom Scenario Setup

```python
# Custom initialization
N = 4  # Number of pursuers
M = 3  # Number of evaders
pursuer_positions = np.array([[0,0,15], [20,0,15], [0,20,15], [20,20,15]])
evader_positions = np.array([[10,10,15], [5,15,15], [15,5,15]])
pursuer_speeds = np.array([1.8, 1.6, 1.7, 1.5])
evader_speeds = np.array([1.0, 1.1, 0.9])

env = Environment(N, M, t, pursuers, evaders)
win_result = env.obtain_trajectories()
```

### Visualization Output

The simulation produces real-time 3D plots showing:
- **Pursuer trajectories** (red lines with circular markers)
- **Evader trajectories** (blue lines with triangular markers)  
- **Goal plane** (green transparent surface at z = 0)
- **Current positions** with capture radius indicators
- **Status information** (step count, active/captured evaders)

### Demo Video


### Performance Analysis

```python
# Extract simulation results
results = {
    'steps': step_count,
    'pursuer_trajectories': pursuer_trajectories,
    'evader_trajectories': evader_trajectories,
    'captured_evaders': captured_evader_indices,
    'escaped_evaders': escaped_evader_indices
}
```

## Usage

### Dependencies

```bash
pip install numpy matplotlib scipy cvxpy
```

### Basic Execution

```bash
cd Research/Code
python Simulation_main.py
```

### Advanced Configuration

Modify parameters in `Simulation_main.py`:

```python
# Game parameters
N = 3  # Number of pursuers  
M = 2  # Number of evaders
t = 0.1  # Time step

# Initial conditions
pursuer_positions = np.array([[0, 0, 12], [26, 0, 12], [15, 13, 12]])
evader_positions = np.array([[15, 0, 12], [16, 7, 12]])
pursuer_speeds = np.array([1.5, 1.5, 1.5])
evader_speeds = np.array([1, 1])
```

### Algorithm Parameters

In `Environment.py`, adjust:

```python
max_coalition_size = 3  # Maximum coalition size
capture_radius = 0.1    # Capture threshold
max_steps = 10000      # Simulation duration
animation_speed = 0.1  # Visualization speed
```

## Research Applications

This implementation has significant applications in:

1. **Autonomous Robotics**
2. **Defense and Security**  
3. **Game Theory Research**
4. **Control Systems**
5. **Operations Research**

## Future Enhancements

### Planned Features
- [ ] Obstacle avoidance integration
- [ ] Machine learning-based strategy optimization

### Research Extensions
- [ ] Calculation of value functions for 1 pursuer vs n evaders
- [ ] Stochastic differential games
- [ ] Multi-objective optimization

## Mathematical Notation

| Symbol | Definition |
|--------|------------|
| `N` | Number of pursuers |
| `M` | Number of evaders |
| `p_i(t)` | Position of pursuer i at time t |
| `e_j(t)` | Position of evader j at time t |
| `v_p,i` | Speed of pursuer i |
| `v_e,j` | Speed of evader j |
| `α_{ij}` | Speed ratio v_p,i / v_e,j |
| `r_c` | Capture radius |
| `V(x,t)` | Value function |
| `T` | Target region |
| `BES` | Boundary of Evasion Space |
| `C_k` | Pursuer coalition k |
| `w(C_i,e_j)` | Matching weight between coalition i and evader j |

## Performance Characteristics

### Computational Complexity
- **Value Function Computation**: O(N³ × M × K) where K is coalition evaluation complexity
- **Matching Algorithm**: O(E³) where E is number of edges in bipartite graph  
- **Overall Per-Step**: O(N³ × M + E³)

### Scalability
- **Recommended limits**: N ≤ 10, M ≤ 8 for real-time performance
- **Memory usage**: O(N³ × M) for value function storage
- **Visualization**: Supports up to 20 total agents with smooth rendering

## References

1. **Matching-based capture strategies for 3D heterogeneous multiplayer reach-avoid differential games** - Base paper for theoretical foundation

2. **Hamilton-Jacobi-Isaacs Equations for Differential Games** - Theoretical framework for value function computation

3. **Multiplayer Reach-Avoid Differential Games in 3D Space** - Multi-player game theory applications  

4. **Weighted Bipartite Matching Algorithms** - Optimization algorithms for coalition-evader matching

5. **Pursuit-Evasion Games: Theory and Applications** - General background on pursuit-evasion dynamics

6. **Coalition Formation in Multi-Agent Systems** - Organizational principles for pursuer coordination

## License

This research code is provided for academic and research purposes. Please cite appropriately if used in publications.

## Contact: 
For research collaboration or technical questions regarding this implementation, please contact ee23b057@smail.iitm.ac.in
---

*This README provides a comprehensive overview of the matching-based capture strategies implementation. For detailed mathematical proofs and extended theoretical analysis, please refer to the academic papers in the `/papers` directory.*
