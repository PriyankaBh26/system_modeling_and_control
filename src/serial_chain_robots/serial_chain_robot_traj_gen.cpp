# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>
#include "serial_chain_robots/serial_chain_robot_traj_gen.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

double CubicTimeScaling(double t_final, double t) {
    double a2 = 3 / std::pow(t_final, 2);
    double a3 = -2 / std::pow(t_final, 3);
    double s = a2 * std::pow(t, 2) + a3 * std::pow(t, 3);
    return s;
}

double QuinticTimeScaling(double t_final, double t) {
    double a3 = 10 / std::pow(t_final, 3);
    double a4 = -15 / std::pow(t_final, 4);
    double a5 = 6 / std::pow(t_final, 5);
    double s = a3 * std::pow(t, 3) + a4 * std::pow(t, 4) + a5 * std::pow(t, 5);
    return s;
}

std::vector<double> JointTrajectory(double q_0, double q_final, 
                                    double t_final, int traj_length, 
                                    std::string time_scaling_type) {

    double dt = t_final/ (traj_length - 1.0);
    std::vector<double> q_traj;                                    
    if (time_scaling_type == "cubic") {
        for (int i{0}; i<traj_length; i++) {
            double s = CubicTimeScaling(t_final, dt * i);
            q_traj.push_back((1 - s)  * q_0 + s * q_final);
        }
    } else if (time_scaling_type == "quintic") {
        for (int i{0}; i<traj_length; i++) {
            double s = QuinticTimeScaling(t_final, dt * i);
            q_traj.push_back((1 - s)  * q_0 + s * q_final);
        }
    }                                 

    return q_traj;
}