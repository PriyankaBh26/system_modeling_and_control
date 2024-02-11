# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/wing_rock_model.h"
# include "data_logging/data_logging_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main () {
    int num_states = 2;

    // initialize reference model ode solver
    VectorXd x0(num_states);
    x0 << 0.0, 0.0;
    double t0 = 0;
    double dh = 1e-4; 

    // nominal plant model matrices A and B
    MatrixXd A(num_states, num_states);
    A << 0, 1,
         0, 0;
    MatrixXd B(num_states, num_states);
    B << 0, 0,
         1, 0;

    // Compute the eigenvalues of A
    Eigen::EigenSolver<MatrixXd> solver1(A);
    Eigen::VectorXcd eigenvalues1 = solver1.eigenvalues();
    std::cout << "A eigenvalues:\n"  << eigenvalues1;

    WingRockModel* system = new WingRockModel(x0, t0, dh, num_states, "wing_rock_model", A, B);
    std::cout << *system;

    VectorXd x_ref(num_states);
    x_ref << 1.0, 0;

    // initialize control input
    VectorXd u(num_states);

    // system dynamics and controller action
    int ode_timesteps = 1000;

    system->IntegrateODE(ode_timesteps, u);

    std::vector<VectorXd> x_history = system->GetXHistory();
    std::vector<double> t_history = system->GetTHistory();

    // save simulation data
    std::string directory = "test/system_models";
    std::string problem = "wing_rock_model";
    SaveSimDataHistory(directory, problem, "state_history", system->GetColumnNames(), x_history);
    SaveTimeHistory(directory, problem, t_history);

    delete system;

    return 0;
}