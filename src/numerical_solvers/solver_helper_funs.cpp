# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "controllers/pidcontroller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


VectorXd CalculateControlInput(VectorXd& x_ref, VectorXd& x, PID* pid_controller, int num_states) {
    VectorXd u(num_states);
    pid_controller->CalculateError(x_ref, x);
    u = pid_controller->GenerateControlInput();
    return u;
}

VectorXd CalculateControlInput(VectorXd& x_ref, int num_states) {
    VectorXd u(num_states);
    double A = 10;
    u = A * x_ref;
    return u;
}
