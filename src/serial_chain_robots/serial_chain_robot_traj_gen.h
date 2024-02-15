#ifndef SERIAL_CHAIN_ROBOT_TRAJ_GEN_H
#define SERIAL_CHAIN_ROBOT_TRAJ_GEN_H

# include <iostream>
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

double CubicTimeScaling(double t_final, double t);

double QuinticTimeScaling(double t_final, double t);

std::vector<double> JointTrajectory(double q_0, double q_final, 
                                    double t_final, int traj_length, 
                                    std::string time_scaling_type);

std::vector<MatrixXd> TfScrewTrajectory(MatrixXd TF_0, MatrixXd TF_final, 
                                        double t_final, int traj_length, 
                                        std::string time_scaling_type);

#endif