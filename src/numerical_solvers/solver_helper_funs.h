#ifndef SOLVER_HELPER_FUNS_H
#define SOLVER_HELPER_FUNS_H

# include <Eigen/Dense>

# include "numerical_solvers/solver_helper_funs.h"
# include "controllers/pidcontroller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd CalculateControlInput(VectorXd& x_ref, VectorXd& x, PID* pid_controller, int num_states);

VectorXd CalculateControlInput(VectorXd& x_ref, int num_states);

void SaveSimulationData(OdeSolver* ode, PID* pid_controller, std::vector<VectorXd>& x_history, std::vector<double> t_history);

void SaveSimulationData(OdeSolver* ode, std::vector<VectorXd>& x_history, std::vector<double> t_history, std::vector<VectorXd>& u_history);


#endif
