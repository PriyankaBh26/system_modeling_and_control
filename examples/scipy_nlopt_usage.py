import numpy as np
from scipy.optimize import minimize

# the constraints are defined as C(x) => 0

# Define the objective function
def objective(x):
    return np.sqrt(x[1])

# Define the constraint function
def ineq_constraint(x, a, b):
    return -(a*x[0] + b)**3 + x[1]

# Define the Jacobian of the objective function
def objective_jacobian(x):
    return [0.0, 0.5 / np.sqrt(x[1])]

# Define the Jacobian of the constraint function
def ineq_constraint_jacobian(x, a, b):
    jac_x0 = -3 * a * (a*x[0] + b)**2
    jac_x1 = np.array([1, 1])
    jac = np.vstack((jac_x0, jac_x1))
    return jac.transpose()

# Initial guess
x0 = np.array([1.234, 5.678])

# Define bounds
bounds = [(None, None), (0, None)]  # non-negative values for both variables
a = np.array([2, -1])
b = np.array([0, 1])

# Define constraint
ineq_constraints = {'type': 'ineq', 
                   'fun': ineq_constraint,
                   'args': (a, b),
                   'jac': ineq_constraint_jacobian}

# Run the optimization
result = minimize(objective, 
                  x0, 
                  method='SLSQP', 
                  bounds=bounds, 
                  constraints=[ineq_constraints], 
                  jac=objective_jacobian,
                  options={'ftol': 1e-9, 'disp': True})

# Print the result
print("Optimal solution:", result.x)
print("Optimal value:", result.fun)
