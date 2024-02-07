# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/dc_motor_velocity.h"
# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main () {

    int num_states = 2; 
    VectorXd x0(num_states);
    x0 << 1.0, 0;
    double t0 = 0.0;
    double dh = 1e-4;

    // initialize DC motor velocity model
    double J = 0.01; // kg.m^2
    double b = 0.1; // N.m.s
    double K = 0.01; // V/rad/s
    double R = 1; // ohm
    double L = 0.5; // H
    DCMotorVelocity* system = new DCMotorVelocity(x0, t0, dh, 
                                                  num_states, "dc_motor_vel", 
                                                  J, b, K, R, L);
    std::cout << *system << "\n";

    // set integration duration
    double time_final = 1;
    int ode_timesteps = time_final/dh;

    // initialize control input
    VectorXd u(1);
    u << 0;

    system->IntegrateODE(ode_timesteps, u);

    std::vector<VectorXd> x_history = system->GetXHistory();
    std::vector<double> t_history = system->GetTHistory();

    std::cout << x_history.size() << " " << x_history[0].size();

    std::string filename = "test/" + system->GetName() + "_solution.csv";
    std::vector<std::string> columnNames = system->GetColumnNames();

    WriteMatToFile(filename, columnNames, x_history);
    
    filename = "test/" + system->GetName() + "_time.csv";
    columnNames = {"time"};
    WriteVecToFile(filename, columnNames, t_history);

    delete system;

    return 0;
}