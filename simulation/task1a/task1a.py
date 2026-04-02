import sympy as sp
import numpy as np
from scipy import linalg

# Define the symbolic variables
x1, x2, u = sp.symbols('x1 x2 u')

# Define the differential equations
x1_dot = -x1 + 2*x1**3 + x2 + 4*u
x2_dot = -x1 - x2 + 2*u


def find_equilibrium_points():
    '''
    Find the equilibrium points by setting the derivatives to zero and solving.
    '''
    # Set x1_dot and x2_dot to zero
    x1_dot_eq = x1_dot.subs(u, 0)
    x2_dot_eq = x2_dot.subs(u, 0)
    
    # Solve the equations for x1 and x2
    equi_points = sp.solve([x1_dot_eq, x2_dot_eq], (x1, x2))

    return equi_points

def find_A_B_matrices(eq_points):
    '''
    Compute the Jacobian matrices A and B at the given equilibrium points.
    '''
    A_matrix = sp.Matrix([
        [sp.diff(x1_dot, x1), sp.diff(x1_dot, x2)],
        [sp.diff(x2_dot, x1), sp.diff(x2_dot, x2)]
    ])
    
    B_matrix = sp.Matrix([
        [sp.diff(x1_dot, u)],
        [sp.diff(x2_dot, u)]
    ])
    
    A_matrices, B_matrices = [], []
    
    # Substitute the equilibrium points into the Jacobian matrices
    for point in eq_points:
        A_matrices.append(A_matrix.subs({x1: point[0], x2: point[1]}))
        B_matrices.append(B_matrix.subs({x1: point[0], x2: point[1]}))
    
    print(A_matrices)
    
    print(B_matrices)
    
    return A_matrices, B_matrices

def find_eigen_values(A_matrices):
    '''
    Find the eigenvalues of the A matrices and determine stability.
    '''
    eigen_values = []
    stability = []

    for A in A_matrices:
        eigenvals = A.eigenvals()
        eigen_values.append(eigenvals)
        # Stability check
        stable = all(val.as_real_imag()[0] < 0 for val in eigenvals.keys())
        stability.append('Stable' if stable else 'Unstable')
 
    return eigen_values, stability

def compute_lqr_gain(jacobians_A, jacobians_B):
    '''
    Compute the LQR gain matrix K using the given Jacobian matrices.
    '''
    # Choose the Jacobian matrices at the first equilibrium point
    A = np.array(jacobians_A[0]).astype(np.float64)
    B = np.array(jacobians_B[0]).astype(np.float64)
    
    # Define the Q and R matrices
    Q = np.eye(2)  # State weighting matrix
    R = np.array([[1]])  # Control weighting matrix

    # Compute the LQR gain using the scipy linalg function
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = np.dot(np.linalg.inv(R), np.dot(B.T, P))
    
    return K

def main_function():
    '''
    Main function to orchestrate the calculations.
    '''
    # Find equilibrium points
    eq_points = find_equilibrium_points()
    
    if not eq_points:
        print("No equilibrium points found.")
        return None, None, None, None, None, None
    
    # Find Jacobian matrices
    jacobians_A, jacobians_B = find_A_B_matrices(eq_points)
    
    # For finding eigenvalues and stability of the given equation
    eigen_values, stability = find_eigen_values(jacobians_A)
    
    # Compute the LQR gain matrix K
    K = compute_lqr_gain(jacobians_A, jacobians_B)
    
    return eq_points, jacobians_A, eigen_values, stability, K

def task1a_output():
    '''
    Print the results obtained from the main function.
    '''
    eq_points, jacobians_A, eigen_values, stability, K = main_function()

    if eq_points is None:
        return
    
    print("Equilibrium Points:")
    for i, point in enumerate(eq_points):
        print(f"  Point {i + 1}: x1 = {point[0]}, x2 = {point[1]}")
    
    print("\nJacobian Matrices at Equilibrium Points:")
    for i, matrix in enumerate(jacobians_A):
        print(f"  At Point {i + 1}:")
        print(sp.pretty(matrix, use_unicode=True))
    
    print("\nEigenvalues at Equilibrium Points:")
    for i, eigvals in enumerate(eigen_values):
        eigvals_str = ', '.join([f"{val}: {count}" for val, count in eigvals.items()])
        print(f"  At Point {i + 1}: {eigvals_str}")
    
    print("\nStability of Equilibrium Points:")
    for i, status in enumerate(stability):
        print(f"  At Point {i + 1}: {status}")
    
    print("\nLQR Gain Matrix K at the selected Equilibrium Point:")
    print(K)

if __name__ == "__main__":
    task1a_output()
