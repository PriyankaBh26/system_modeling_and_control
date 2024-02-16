# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/mass_spring_damper.h"
# include "data_logging/data_logging_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main () {

    int num_states = 2; 
    VectorXd x0(num_states);
    x0 << 1.0, 0;
    double t0 = 0.0;
    double dh = 1e-4;

    // initialize DC motor velocity model
    double k = 0.01; // N/m
    double c = 0.1; // N/m.s
    double m = 0.01; // Kg

    MassSpringDamperSys* system = new MassSpringDamperSys(x0, t0, dh, 
                                                  num_states, "mass_spring_damper", 
                                                  k, c, m);
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

    // save simulation data 
    std::string directory = "test/system_models/";
    std::string problem = system->GetName();
    std::string data_type = "solution";
    std::vector<std::string> column_names = system->GetColumnNames();
 
    SaveSimDataHistory(directory,
                        problem,
                        data_type,
                        column_names,
                        x_history,
                        "replace");
    
    SaveTimeHistory(directory, 
                    problem, 
                    t_history,
                    "replace");
    delete system;

    return 0;
}