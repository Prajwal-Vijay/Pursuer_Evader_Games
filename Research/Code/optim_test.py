import cvxpy as cp
import numpy as np
from scipy.optimize import Bounds
from scipy.optimize import minimize

constraints = []
evader_pos = np.array([[1], [2], [8]])
pursuer_pos = np.array([[1], [2], [1]])
alpha_ij = 2.25
capture_radius = 0.1
print(evader_pos.shape)
def objective(x):
    return x[2]

# Constraint: dist_to_pursuer^2 - alpha_ij * dist_to_evader^2 >= 0
def constraint_fun(x):
    dist_to_pursuer = np.linalg.norm(x - pursuer_pos)
    dist_to_evader = np.linalg.norm(x - evader_pos)
    return dist_to_pursuer - alpha_ij * dist_to_evader - capture_radius

# Capture radius constraint: dist_to_pursuer >= capture_radius
# def capture_constraint(x):
#     return np.linalg.norm(x - pursuer_pos) - capture_radius

# Put constraints in dictionary form
constraints = [
    {'type': 'ineq', 'fun': constraint_fun}
]

# Initial guess
x0 = np.array([[0],[0],[0]])

# Solve
res = minimize(objective, x0, constraints=constraints)

print("Optimal x:", res.x)
print("Success:", res.success)
print("Message:", res.message)
