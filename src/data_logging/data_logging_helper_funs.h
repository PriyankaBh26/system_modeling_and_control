#ifndef DATA_LOGGING_HELPER_FUNS_H
#define DATA_LOGGING_HELPER_FUNS_H

# include <Eigen/Dense>
# include "numerical_solvers/rk_ode_solver.h"
# include "controllers/pidcontroller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void SaveSimulationData(OdeSolver* ode, 
                        PID* pid_controller, 
                        std::vector<VectorXd>& x_history, 
                        std::vector<double> t_history);

void SaveSimulationData(OdeSolver* ode, 
                        std::vector<VectorXd>& x_history, 
                        std::vector<double> t_history, 
                        std::vector<VectorXd>& u_history);

void SaveKFSimulationData(OdeSolver* ode, 
                          std::vector<VectorXd>& x_history, 
                          std::vector<double> t_history, 
                          std::vector<VectorXd>& x_est_history, 
                          std::vector<VectorXd>& z_history);

void SaveKFPIDSimulationData(OdeSolver* ode, 
                        std::vector<VectorXd>& x_history, 
                        std::vector<double> t_history, 
                        std::vector<VectorXd>& x_est_history, 
                        std::vector<VectorXd>& z_history,
                        std::vector<VectorXd>& u_history);

#endif