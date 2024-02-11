import numpy as np
from scipy import linalg

def SolveLyapunovEquation(A):
    """
    Computes a solution (X) to the Sylvester equation:
    A X + X B = Q

    The corresponding Lyapunov equation is: 
    A.T P + P A = -I

    Parameters: 
    A = n x n matrix
    Returns:
    P = n x n symmetric matrix 
    """
    n = A.shape[0]
    Q = - np.eye(n)
    P = linalg.solve_sylvester(A.T, A, Q)
    return P

def main():
    A = np.array([[0, 1], [-4, -2]])
    P = SolveLyapunovEquation(A)
    print(P)

if __name__ == "__main__":
    main()