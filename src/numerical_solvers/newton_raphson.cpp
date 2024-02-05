# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "newton_raphson.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


NewtonRaphson::NewtonRaphson(int n, int m, VectorXd q0, 
                            double eps, int N0) : num_states(n), num_equations(m), q(q0), 
                            epsilon(eps), max_iterations(N0) {q_history.push_back(q0);
                                                                success = false;};

// the equation f(q) = 0
virtual VectorXd NewtonRaphson::f(VectorXd q) {return VectorXd v(num_equations)};
// the jacobian = dfdq(q)
virtual MatrixXd NewtonRaphson::dfdq(VectorXd q) {return MatrixXd mat(num_equations, num_states)};
// iteration q = q0 - pinv(dfdq(q0)) * f(q0);
VectorXd NewtonRaphson::iterate() {
    double error = f(q);
    int num_iterations = 0;
    while (error.norm() > epsilon & num_iterations < max_iterations) {
        // compute jacobian
        MatrixXd jacobian = dfdq(q);
        // Compute the pseudo-inverse
        MatrixXd pinv_j = Eigen::CompleteOrthogonalDecomposition<MatrixXd>(jacobian).pseudoInverse();
        // update q
        q = q + pinv_j * error;
        // save q history
        q_history.push_back(q);
        // increment number of iterations
        num_iterations += 1;
    }
    NewtonRaphson::convergence_check(double error, int num_iterations);
    return q;
};

std::vector<VectorXd> NewtonRaphson::get_q_history() {
    return q_history; 
}

void NewtonRaphson::convergence_check(double error, int num_iterations) {
    if (error < epsilon) {
        std::cout << "Successfully achieved convergence!\n"; 
        std::cout << "Number of iterations = " << num_iterations << "\n";
        std::cout << "Final error = " << error << "\n";
        success = true;
    } else if (error > epsilon && num_iterations > max_iterations) {
        std::cout << "Failure to converge!\n";
        std::cout << "Maximum number of iterations "  << max_iterations << " reached.\n"
        std::cout << "Final error = " << error << "\n";
    } else {
        std::cout << "Successfully achieved convergence!\n"; 
        std::cout << "Number of iterations = " << num_iterations << "\n";
        std::cout << "Final error = " << error << "\n";        
        success = true;
    }
}
