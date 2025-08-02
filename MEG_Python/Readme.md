<<<<<<< HEAD
# Pursuer-Evader Games (Python Implementation)

A Python implementation of multi-agent pursuit-evasion differential games with geometric control theory and heuristic algorithms. This project simulates strategic interactions between pursuers and evaders in 2D space using advanced mathematical concepts from differential game theory.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Mathematical Foundation](#mathematical-foundation)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Game Mechanics](#game-mechanics)
- [Algorithms](#algorithms)
- [Examples](#examples)
- [Contributing](#contributing)
- [References](#references)

## Overview

This project implements a sophisticated pursuit-evasion game where a single pursuer attempts to capture multiple evaders while the evaders try to reach a target position. The simulation uses principles from:

- **Differential Game Theory** - Mathematical framework for multi-agent strategic interactions
- **Barrier Functions** - Determining win conditions and game feasibility
- **Geometric Control Theory** - Using geometric constructs like Apollonius circles for optimal strategies
- **Heuristic Algorithms** - Intelligent decision-making for multi-agent coordination

## Features

- **Multi-Agent Simulation**: Support for multiple evaders with heterogeneous capabilities
- **Advanced Game Theory**: Implementation of barrier functions and win condition analysis
- **Geometric Strategies**: Apollonius circle-based escape strategies for evaders
- **Heuristic Algorithms**: Intelligent pursuer strategies including:
  - Weighted average of optimal headings
  - Closest evader pursuit
- **Real-time Visualization**: Dynamic plotting of agent positions and trajectories
- **Configurable Parameters**: Adjustable speeds, positions, and game parameters

## Mathematical Foundation

### Barrier Function
The core mathematical concept determining game feasibility:

```
B_ij = ||target - evader_j||² - α_j² * ||target - pursuer||²
```

Where `α_j = evader_speed_j / pursuer_speed` is the speed ratio.

### Win Condition
- If `min(B_ij) ≥ 0` for all evaders, the pursuer has a potential winning strategy
- If `min(B_ij) < 0`, at least one evader can guarantee escape regardless of pursuer strategy

### Geometric Control
- **Apollonius Circles**: Define regions where evaders can guarantee escape
- **Perpendicular Bisector Method**: Used when speed ratios equal 1
- **Optimal Heading Calculations**: Mathematical optimization for intercept strategies

## Installation

### Prerequisites

```
Python 3.7+
numpy
matplotlib
scipy
```

### Setup

1. Clone the repository:
```bash
git clone https://github.com/Prajwal-Vijay/Pursuer_Evader_Games.git
cd Pursuer_Evader_Games/MEG_Python
```

2. Install dependencies:
```bash
pip install numpy matplotlib scipy
```

## Usage

### Basic Example

```python
import numpy as np
import Environment

# Game parameters
N = 2  # 2D space
n = 2  # Number of evaders
v = 1  # Pursuer speed
u = np.ones(n)  # Evader speeds
r = 0.01  # Time step

# Initial positions
pursuer_position = np.array([[-1.7720], [0.3751]])
evader_positions = np.array([[9.1501, -6.8477], [9.2978, 9.4119]])
target_position = np.array([[0], [0]])

# Create environment
env = Environment.Environment(N, n, v, u, r, pursuer_position, evader_positions, target_position)

# Check game feasibility
win = env.check_initialization(env.evaders, True)

if win:
    print("Running simulation")
    win_result, pursuer_traj, evader_traj = env.obtain_trajectories('heuristic')
    if win_result:
        print('Pursuer wins!')
    else:
        print('Evaders win!')
else:
    print('Evaders guaranteed to win - unfair game!')
```

### Running the Simulation

```bash
python main.py
```

## Project Structure

```
MEG_Python/
├── main.py              # Entry point and example usage
├── Environment.py       # Game environment and simulation logic
├── Pursuer.py          # Pursuer agent with optimization strategies
├── Evader.py           # Evader agent with escape strategies
└── MEGDocumentation.pdf # Detailed mathematical documentation
```

### Core Classes

#### Environment
- Manages game state and agent interactions
- Handles barrier function calculations
- Controls simulation stepping and termination
- Provides visualization capabilities

#### Pursuer
- Implements multiple pursuit strategies
- Calculates optimal headings using geometric methods
- Provides heuristic and optimization-based algorithms
- Supports both single and multi-evader scenarios

#### Evader
- Implements escape strategies based on game theory
- Uses Apollonius circles for optimal escape paths
- Adapts behavior based on speed ratios and win conditions
- Geometric path planning for target reaching

## Game Mechanics

### Initialization Phase
1. **Barrier Calculation**: Compute barrier values for all evaders
2. **Feasibility Check**: Determine if pursuer has winning potential
3. **Strategy Selection**: Choose appropriate algorithms based on game state

### Simulation Loop
1. **Position Updates**: Move all agents according to their strategies
2. **Collision Detection**: Check for captures using distance thresholds
3. **Target Checking**: Monitor if evaders reach the target
4. **Termination**: End game when capture occurs or evader escapes

### Termination Conditions
- **Pursuer Victory**: All evaders captured
- **Evader Victory**: Any evader reaches target without capture
- **Stalemate**: Maximum simulation time exceeded

## Algorithms

### Pursuer Strategies

#### Heuristic Algorithm
Uses weighted average of optimal headings toward all evaders:
- Calculates optimal intercept angle for each evader
- Weights decisions based on capture difficulty
- Provides robust multi-evader handling

#### Closest Evader Strategy
Simple but effective approach:
- Direct pursuit of nearest evader
- Computationally efficient
- Suitable for scenarios with speed advantages

### Evader Strategies

#### Equal Speed Ratio (α = 1)
- Uses perpendicular bisector method
- Finds intersection points for optimal escape
- Geometric optimization for target approach

#### Unequal Speed Ratio (α < 1)
- Employs Apollonius circle construction
- Defines escape regions mathematically
- Advanced geometric path planning

## Examples

### Multi-Evader Scenario
```python
# Setup with 3 evaders of different speeds
n = 3
u = np.array([0.8, 1.0, 1.2])  # Different evader speeds
evader_positions = np.array([[2, -3, 4], [1, 2, -1]])

env = Environment.Environment(N, n, v, u, r, pursuer_position, evader_positions, target_position)
```

### Custom Visualization
```python
# Plot current game state
env.plot_current_positions()
plt.title('Pursuit-Evasion Game State')
plt.show()
```

## Configuration Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `N` | Motion space dimension | 2 |
| `n` | Number of evaders | 2 |
| `v` | Pursuer speed | 1.0 |
| `u` | Evader speeds (array) | [1.0, 1.0] |
| `r` | Time step size | 0.01 |
| `capture_tolerance` | Capture radius | 0.05 |

## Applications

This framework can be applied to various real-world scenarios:

- **Robotics**: Multi-robot coordination and interception
- **Surveillance**: Security systems and threat assessment  
- **Autonomous Vehicles**: Traffic management and collision avoidance
- **Military**: Tactical planning and engagement strategies
- **Game Development**: AI behavior for strategy games

## Contributing

Contributions are welcome! Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## References

1. Isaacs, R. (1965). *Differential Games*. John Wiley & Sons.
2. Basar, T., & Olsder, G. J. (1999). *Dynamic Noncooperative Game Theory*. SIAM.
3. Weintraub, I. E., Pachter, M., & Garcia, E. (2020). An introduction to pursuit-evasion differential games. *American Control Conference*.
4. Dorothy, M., et al. (2022). One Apollonius circle is enough for many pursuit-evasion games. *Automatica*.
5. Bhattacharya, S., & Hutchinson, S. (2010). On the existence of Nash equilibrium for a two-player pursuit-evasion game. *International Journal of Robotics Research*.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Original MATLAB implementation and mathematical framework
- Research in differential game theory and geometric control
- Open-source scientific computing libraries (NumPy, SciPy, Matplotlib)

## Contact

For questions, suggestions, or collaboration opportunities, please open an issue or contact the maintainers.

---

*This implementation bridges theoretical differential game concepts with practical multi-agent simulations, providing a powerful tool for research and education in autonomous systems and game theory.*
=======
# Introduction
This is a python implementation of the following repo [MEG](https://github.com/abinashagasti/Multiplayer_Reach_Avoid/tree/master/MEG). 
I have also documented it [here](https://github.com/Prajwal-Vijay/Pursuer_Evader_Games/blob/main/MEG_Python/MEGDocumentation.pdf).

Here we solve scenarios of single pursuer vs n evaders.
## Demo
https://github.com/user-attachments/assets/24520da7-1df7-47e9-89b9-b8fea71b0586

>>>>>>> 2c3b2fe7c8c305c585d365603224539c4d12522a
