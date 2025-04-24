import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb

def bernstein_poly(n, i, t):
    # Compute the Bernstein polynomial value at t
    #   n: total number of control points minus one
    #   i: the index of the control point
    #   t: curve parameter (ranges from 0 to 1)
    #   comb(n, i): binomial coefficient
    return comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

def bezier_curve(control_points, num_samples=100):
    # Compute a Bézier curve from given control points

    n = len(control_points) - 1   # number of control points minus 1
    t_values = np.linspace(0, 1, num_samples)   # curve parameter
    curve = np.zeros((num_samples, 2))   # initialize curve array
    
    # Update curve
    for i in range(n + 1):
        curve += np.outer(bernstein_poly(n, i, t_values), control_points[i])
    
    return curve, curve[:, 0], curve[:, 1]

def plot_trajectory(control_points, curve):
    # Plot the Bézier curve and control points
    plt.figure(figsize=(8, 6))
    plt.plot(curve[:, 0], curve[:, 1], 'b-', label='Bézier Curve')
    plt.plot(control_points[:, 0], control_points[:, 1], 'ro--', label='Control Points')
    
    for i, (x, y) in enumerate(control_points):
        plt.text(x, y, f'P{i}', fontsize=12, verticalalignment='bottom', horizontalalignment='right')
    
    # plt.xlim(-13, 13)
    # plt.ylim(-13, 13)
    plt.xlim(-6, 6)
    plt.ylim(-6, 6)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Quadruped Foot Trajectory (Bézier Curve)')
    plt.legend()
    plt.grid()
    plt.show()

# Foot range x = y = [-12, 12]

# Define standing control Points
# control_points = np.array([
#     [0, 3.5], [1, -7], [0, -12]
# ])

# Define sitting control Points
# control_points = np.array([
#     [0, -12], [1, -7], [0, 3.5]
# ])

# Define walking control points
control_points = np.array([
    [-2.5, 0], [-4, 0], [-5, 2], [-5, 2], [-5, 2], [0, 2], 
    [0, 2], [0, 2.5], [5, 2.5], [5, 2.5], [4, 0], [2.5, 0],
    [2.5, 0], [0, -0.5], [-2.5, 0]
])

# Compute the Bézier curve
curve, x_pos, y_pos = bezier_curve(control_points, num_samples=200)
# print("X Coordinates:", x_pos)
# print("Y Coordinates:", y_pos)

# Plot the result
plot_trajectory(control_points, curve)
