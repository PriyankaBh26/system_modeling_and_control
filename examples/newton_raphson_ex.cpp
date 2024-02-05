# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "numerical_solvers/newton_raphson.h"
# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FindRoot: public NewtonRaphson {
    public:
        FindRoot(int num_states, int num_equations, VectorXd q0, 
                double epsilon, int max_iterations) : NewtonRaphson(num_states, num_equations, q0, 
                                            epsilon, max_iterations) {};
        VectorXd f(VectorXd q) override {
            VectorXd v(1);
            v << q(0)*q(0) - 1.0;
            return v;};
        
        MatrixXd dfdq(VectorXd q) override {
            MatrixXd mat(1, 1);
            mat << 2*q(0);
            return mat;};

};

int main() {
    int num_states = 1;
    int num_equations = 1;
    VectorXd q0(1);
    q0 << 0.5;
    double epsilon = 1e-3;
    int max_iterations = 50;

    NewtonRaphson* eq1 = new FindRoot(num_states, num_equations, q0, 
                                            epsilon, max_iterations);

    eq1->Iterate();
    std::vector<VectorXd> q_history = eq1->GetSolutionHistory();
    std::cout << q_history[0] << "\n";
    delete eq1;
    return 0;
}