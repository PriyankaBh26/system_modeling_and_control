# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "numerical_solvers/newton_raphson.h"
# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class FindRoots: public NewtonRaphson {
    public:
        VectorXd NewtonRaphson::f(VectorXd q) override {return VectorXd v(num_equations)};
        
        MatrixXd NewtonRaphson::dfdq(VectorXd q) {return MatrixXd mat(num_equations, num_states)};

};