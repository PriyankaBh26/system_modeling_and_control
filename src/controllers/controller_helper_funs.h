#ifndef CONTROLLER_HELPER_FUNS_H
#define CONTROLLER_HELPER_FUNS_H

# include <Eigen/Dense>

# include "controllers/pid_controller.h"
# include "controllers/ackermans_formula_pole_placement.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void CheckSysStability(MatrixXd A, std::string system_type);

VectorXd FindKAckermanFormula(MatrixXd A, MatrixXd B, VectorXd coeffs, std::string ip_vector_type);

VectorXd ScaleCLTransferFunction(MatrixXd A, MatrixXd B, MatrixXd C, VectorXd K, double r_ss);

#endif