#ifndef SOLVER_HELPER_FUNS_H
#define SOLVER_HELPER_FUNS_H

# include <Eigen/Dense>

# include "numerical_solvers/solver_helper_funs.h"
# include "controllers/pidcontroller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd CalculateControlInput(VectorXd& x_ref, VectorXd& x, PID* pid_controller, int num_states);

VectorXd CalculateControlInput(VectorXd& x_ref, int num_states);

#endif
