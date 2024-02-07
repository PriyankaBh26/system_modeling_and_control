# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/van_der_pol_oscillator.h"
# include "controllers/pid_controller.h"
# include "data_logging/savecsv.h"
# include "data_logging/data_logging_helper_funs.h"
# include "state_estimators/unscented_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

static const double MU_1 = 5.5;

// update state variables, f(x) = [x1dot, x2dot .. xndot]T
VectorXd UnscentedKalmanFilter::f(VectorXd Y) {
    int n = GetN();
    double dt = GetDt();
    MatrixXd ydot(n, 1);
    ydot << Y[1], MU_1 * (1 - std::pow(Y[0], 2)) * Y[1] - Y[0];
    
    Y = Y + ydot*dt;
    return Y;
};

// update outputs, y = h(x)
VectorXd UnscentedKalmanFilter::h(VectorXd Y) {
    int n = GetN();
    int m = GetM();
    VectorXd Z(n);
    MatrixXd H(m, n);
    H << 1, 0,
        0, 1;
    Z = H * Y;
    return Z;
};

int main() {
    int num_states = 2;
    int num_outputs = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 0.0, 5.0;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 1, 0.0,
          0.0, 1;

    // define mass-spring-damper system variables
    double t0 = 0.0;
    double dh = 1e-4;

    VanDerPolOscillator* system = new VanDerPolOscillator(x0, t0, dh);

    // set integration duration
    double dt = 1e-2; 
    double time_final = 10.0;

    // define process noise matrix
    MatrixXd Q(num_states, num_states);
    Q << 1e-1, 0.0,
          0.0, 1e-1;
    // define measurement noise matrix
    MatrixXd R(num_states, num_outputs);
    R << 1e-1, 0.0,
          0.0, 1e-1;

    // initialize kalman filter object
    VectorXd x0_m(num_states);
    x0_m << 1.0, 0.0;

    // set kappa to compute weights
    double kappa = abs(3 - num_states);

    UnscentedKalmanFilter* ukf = new UnscentedKalmanFilter(x0_m, P0, Q, 
                                                          R, dt, num_states, 
                                                          num_outputs, kappa);

    // initialize measurement z
    std::vector<VectorXd> z_history;

    // initialize control input
    VectorXd u(num_states);

    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_est_history;
    std::vector<double> t_history;

    // system dynamics and controller action
    double t = 0;
    while (t < time_final) {
        int ode_timesteps = dt/dh;

        VectorXd x = system->GetX();
        
        z_history.push_back(x + R * VectorXd::Random(num_states));

        VectorXd x_est = ukf->ComputeEstimate(z_history.back());

        system->IntegrateODE(ode_timesteps, u);

        t += dt;
        x_history.push_back(x);
        x_est_history.push_back(x_est);
        t_history.push_back(t);
    }
    // save simulation data for plotting
    SaveKFSimulationData(system, x_history, t_history, x_est_history, z_history);

    delete ukf;
    delete system;
    return 0;
}