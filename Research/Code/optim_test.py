import cvxpy as cp
import numpy as np

x = cp.Variable(shape=(3,1))
constraints = []
evader_pos = np.array([[1], [2], [5]])
pursuer_pos = np.array([[1], [2], [1]])
print(evader_pos.shape)
alpha_ij = 1.5
capture_radius = 0.1
dist_to_pursuer = cp.norm(x - pursuer_pos)
dist_to_evader = cp.norm(x - evader_pos)
constraints.append(dist_to_pursuer >= alpha_ij*dist_to_evader + capture_radius)
objective = cp.Minimize(x[2,0])
problem = cp.Problem(objective, constraints)
problem.solve(solver=cp.ECOS_BB, verbose=False)
print(problem.value)