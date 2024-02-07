# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "system_models/van_der_pol_oscillator.h"
# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/mass_spring_damper.h"
# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main () {
    
    int num_states = 2; 
    VectorXd x0(num_states);
    x0 << 1.0, 0;
    double t0 = 0.0;
    double dh = 0.0001;

    // initialize mass-spring-damper system
    // double k = 1.0;
    // double c = 1.0;
    // double m = 1.0;
    // MassSpringDamperSys* ode = new MassSpringDamperSys(x0, t0, dh, num_states, "msd", k, c, m);

    // initialize van der pol oscillator system
    double mu = 2.5;
    VanDerPolOscillator* ode = new VanDerPolOscillator(x0, t0, dh, num_states, "van_der_pol", mu);

    // set integration duration
    double time_final = 1;
    int ode_timesteps = time_final/dh;

    // initialize control input
    VectorXd u(1);
    u << 0;

    ode->IntegrateODE(ode_timesteps, u);

    std::vector<VectorXd> x_history = ode->GetXHistory();
    std::vector<double> t_history = ode->GetTHistory();

    std::cout << x_history.size() << " " << x_history[0].size();

    std::string filename = "examples/" + ode->GetName() + "_solution.csv";
    std::vector<std::string> columnNames = {"Pos", "Vel"};

    WriteMatToFile(filename, columnNames, x_history);
    
    filename = "examples/" + ode->GetName() + "_time.csv";
    columnNames = {"time"};
    WriteVecToFile(filename, columnNames, t_history);

    delete ode;
    return 0;
}