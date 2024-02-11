# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/van_der_pol_oscillator.h"
# include "controllers/pid_controller.h"
# include "data_logging/data_logging_helper_funs.h"
# include "state_estimators/extended_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

static const double MU_1 = 3.0;

// update state variables, f(x) = [x1dot, x2dot .. xndot]T
VectorXd ExtendedKalmanFilter::f() {
    MatrixXd xdot(n, 1);
    xdot << x[1], MU_1 * (1 - std::pow(x[0], 2)) * x[1] - x[0];
    
    x = x + xdot*dt;
    return x;
};

// update outputs, y = h(x)
VectorXd ExtendedKalmanFilter::h() {
    VectorXd y(n);
    MatrixXd H(m, n);
    H << 1, 0,
        0, 1;
    y = H * x;
    return y;
};

// calculate f jacobian = [df/dx1 df/dx2 .. df/dxn]
MatrixXd ExtendedKalmanFilter::CalculateFxJacobian() {
    MatrixXd A(n, n);
    A << 0, 1, 
        MU_1 * 2 * x[0] * x[1] - 1, MU_1 *  (1 - std::pow(x[0], 2));
    A = MatrixXd::Identity(n, n) + A * dt;
    return A;
};

// calculate h jacobian H 
MatrixXd ExtendedKalmanFilter::CalculateHxJacobian() {
    MatrixXd H(m, n);
    H << 1, 0,
        0, 1;
    return H;
};


int main() {
    int num_states = 2;
    int num_outputs = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 10.0, 5.0;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 1, 0.0,
          0.0, 1;

    // initialize control input
    VectorXd u(1);

    // define Van der Pol oscillator system variables
    double t0 = 0.0;
    double dh = 1e-4;

    double mu = 2.5;
    VanDerPolOscillator* system = new VanDerPolOscillator(x0, t0, dh,
                                                         num_states, "van_der_pol", mu);
    std::cout << *system;

    // set integration duration
    double dt = 1e-2; 
    double time_final = 100.0;

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
    x0_m << 0.0, 0.0;
    ExtendedKalmanFilter* ekf = new ExtendedKalmanFilter(x0_m, P0, 
                                                        Q, R, 
                                                        dt, num_states, num_outputs);

    // initialize measurement z
    std::vector<VectorXd> meas_history;

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
        
        meas_history.push_back(x + R * VectorXd::Random(num_states));

        VectorXd x_est = ekf->ComputeEstimate(meas_history.back());

        t += dt;
        x_history.push_back(x);
        x_est_history.push_back(x_est);
        t_history.push_back(t);
    }
    // save simulation data for plotting
    std::string directory = "examples";
    std::string problem = "ekf";
    SaveTimeHistory(directory, problem, t_history);
    SaveSimDataHistory(directory, problem, "state_history", system->GetColumnNames(), x_history);
    SaveSimDataHistory(directory, problem, "meas_history", system->GetColumnNames(), meas_history);
    SaveSimDataHistory(directory, problem, "est_history", system->GetColumnNames(), x_est_history);

    delete ekf;
    delete system;
    return 0;
}