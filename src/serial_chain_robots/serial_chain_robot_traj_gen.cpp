# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>
#include "serial_chain_robots/serial_chain_robot_traj_gen.h"
# include "serial_chain_robots/serial_chain_robot_helper_funs.h"

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

double UpdateTimeScaling(std::string time_scaling_type, double t_final, double t) {
    double s = 0;
    if (time_scaling_type == "cubic") {
        s = CubicTimeScaling(t_final, t);
    } else if (time_scaling_type == "quintic")  {
        s = QuinticTimeScaling(t_final, t);
    }
    return s;
}

std::vector<double> JointTrajectory(double q_0, double q_final, 
                                    double t_final, int traj_length, 
                                    std::string time_scaling_type) {
    double dt = t_final/ (traj_length - 1.0);
    std::vector<double> q_traj;                                    
    for (int i{0}; i<traj_length; i++) {
        double s = UpdateTimeScaling(time_scaling_type, t_final, dt * i);
        q_traj.push_back((1 - s)  * q_0 + s * q_final);
    }                             

    return q_traj;
}

std::vector<MatrixXd> TfScrewTrajectory(MatrixXd TF_0, MatrixXd TF_final, 
                                        double t_final, int traj_length, 
                                        std::string time_scaling_type) {
    double dt = t_final/ (traj_length - 1.0);
    std::vector<MatrixXd> TF_traj;
    for (int i{0}; i<traj_length; i++) {
        double s = UpdateTimeScaling(time_scaling_type, t_final, dt * i);
        MatrixXd TF_s = MatrixExp6(MatrixLog6(TfmatInverse(TF_0) * TF_final) * s);
        MatrixXd TF = TF_0 * TF_s;
        TF_traj.push_back(TF);
    }
    
    return TF_traj;
}

std::vector<MatrixXd> TfCartesianTrajectory(MatrixXd TF_0, MatrixXd TF_final, 
                                        double t_final, int traj_length, 
                                        std::string time_scaling_type) {
    double dt = t_final/ (traj_length - 1.0);
    MatrixXd R_0 = TF_0.block(0,0,3,3);
    VectorXd p_0 = TF_0.block(0,3,3,1);

    MatrixXd R_final = TF_final.block(0,0,3,3);
    VectorXd p_final = TF_final.block(0,3,3,1);

    std::vector<MatrixXd> TF_traj;
    for (int i{0}; i<traj_length; i++) {
        double s = UpdateTimeScaling(time_scaling_type, t_final, dt * i);

        MatrixXd R_i = MatrixLog3(R_0.transpose() * R_final) * s;

        MatrixXd R_s = R_0 * MatrixExp3(R_i);
        VectorXd p_s = (1 - s)  * p_0 + s * p_final;

        MatrixXd TF(4,4);
        TF.block(0,0,3,3) = R_s;
        TF.block(0,3,3,1) = p_s;
        TF(3,3) = 1.0;

        TF_traj.push_back(TF);
    }

    return TF_traj;
}
