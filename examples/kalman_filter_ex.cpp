# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/mass_spring_damper.h"
# include "data_logging/savecsv.h"
# include "data_logging/data_logging_helper_funs.h"
# include "state_estimators/kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    int num_states = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 0.0, 5;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 10, 0.0,
          0.0, 10;

    // define mass-spring-damper system variables
    double t0 = 0.0;
    double dh = 1e-4;

    // initialize mass-spring-damper system
    double k = 1.0;
    double c = 1.0;
    double m = 1.0;
    MassSpringDamperSys* system = new MassSpringDamperSys(x0, t0, dh, num_states, "msd", k, c, m);

    // set integration duration
    double dt = 1e-2; 
    double time_final = 10.0;

    // define state matrix A for discrete system
    MatrixXd A_state(num_states, num_states);
    A_state = MatrixXd::Identity(num_states, num_states) + system->GetA() * dt;

    // define process noise matrix
    MatrixXd Q(num_states, num_states);
    Q << 1e-2, 0.0,
          0.0, 1e-2;
    // define output matrix
    MatrixXd H(num_states, num_states);
    H << 1.0, 0.0,
         0.0, 1.0;
    // define measurement noise matrix
    MatrixXd R(num_states, num_states);
    R << 1e-1, 0.0,
          0.0, 1e-2;

    // initialize kalman filter object
    VectorXd x0_m(num_states);
    x0_m << 0.0, 0.0;
    KalmanFilter* kf = new KalmanFilter(x0_m, P0, A_state, Q, H, R);

    // initialize measurement z
    std::vector<VectorXd> z_history;

    // initialize control input
    VectorXd u(1);

    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_est_history;
    std::vector<double> t_history;

    // system dynamics and controller action
    double t = 0;
    while (t < time_final) {
        int ode_timesteps = dt/dh;
        system->IntegrateODE(ode_timesteps, u);

        VectorXd x = system->GetX();
        
        z_history.push_back(x + R * VectorXd::Random(num_states));

        VectorXd x_est = kf->ComputeEstimate(z_history.back());

        t += dt;
        x_history.push_back(x);
        x_est_history.push_back(x_est);
        t_history.push_back(t);
    }
    // save simulation data for plotting
    SaveKFSimulationData(system, x_history, t_history, x_est_history, z_history);

    delete kf;
    delete system;
    return 0;
}