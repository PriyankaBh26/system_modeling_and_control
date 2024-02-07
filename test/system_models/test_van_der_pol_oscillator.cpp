# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/van_der_pol_oscillator.h"
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
    double mu = 2.5;

    VanDerPolOscillator* system = new VanDerPolOscillator(x0, t0, dh, 
                                                  num_states, "van_der_pol", 
                                                  mu);
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