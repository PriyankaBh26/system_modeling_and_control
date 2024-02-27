# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "system_models/simple_pendulum.h"
# include "controllers/pid_controller.h"
# include "data_logging/data_logging_helper_funs.h"
# include "state_estimators/extended_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

double m_1 = 1.0;
double b_1 = 1;
double l_1 = 1;

// update state variables, f(x) = [x1dot, x2dot .. xndot]T
VectorXd ExtendedKalmanFilter::f(VectorXd Bu) {
    VectorXd xd(n);
    xd << x(1), -9.81/l_1 * sin(x(0)) - b_1/(m_1*l_1*l_1) * x(1);
    
    xd += Bu/(m_1*l_1*l_1);
    
    x = x + xd*dt;
    return x;
};

// update outputs, y = h(x)
VectorXd ExtendedKalmanFilter::h() {
    VectorXd y(n);
    MatrixXd H(m, n);
    H << 1, 0,
        0, 0;
    y = H * x;
    return y;
};

// calculate f jacobian = [df/dx1 df/dx2 .. df/dxn]
MatrixXd ExtendedKalmanFilter::CalculateFxJacobian() {
    MatrixXd A(n, n);
    A << 0, 1,
         -9.81/l_1 * cos(x(0)), -b_1/(m_1*l_1*l_1) ;
    A = MatrixXd::Identity(n, n) + A * dt;
    return A;
};

// calculate h jacobian H 
MatrixXd ExtendedKalmanFilter::CalculateHxJacobian() {
    MatrixXd H(m, n);
    H << 1, 0,
        0, 0;
    return H;
};


int main() {
    int num_states = 2;
    int num_outputs = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 0.0, 0.0;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 1, 0.0,
          0.0, 1;

    double t0 = 0.0;
    double dh = 1e-4;

    // initialize simple pendulum model
    double m = 1;
    double b = 1;
    double l = 1;
        
    SimplePendulum* system = new SimplePendulum(x0, t0, dh,
                                                num_states, "simple_pendulum",
                                                m, b, l);
    std::cout << *system;

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
    x0_m << 0.0, 0.0;
    ExtendedKalmanFilter* ekf = new ExtendedKalmanFilter(x0_m, P0, 
                                                        Q, R, 
                                                        dt, num_states, num_outputs);

    // intialize pid controller
    VectorXd K(2);
    K << 3, 1.0;
    PID* pid_controller = new PID(1);
    MatrixXd KP(1,1);
    KP(0,0) = K(0);
    
    MatrixXd KI(1,1);
    KI(0,0) = 0.2;

    MatrixXd KD(1,1);
    KD(0,0) = K(1);

    pid_controller->SetGains(KP, KI, KD);

    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_ref_history;
    std::vector<VectorXd> x_est_history;
    std::vector<VectorXd> x_err_history;
    std::vector<VectorXd> meas_history;
    std::vector<double> t_history;

    double t = 0;
    x_history.push_back(x0);
    x_ref_history.push_back(x0);
    x_est_history.push_back(x0);
    x_err_history.push_back(x0 - x0);
    meas_history.push_back(x0);
    t_history.push_back(t);

    // initialize control input
    MatrixXd B = system->GetB();
    VectorXd u(1);

    // choose reference trajectory 
    VectorXd x_ref(num_states);
    x_ref << 1, 0;

    // system dynamics and controller action
    while (t < time_final) {
        int ode_timesteps = dt/dh;
        system->IntegrateODE(ode_timesteps, u);

        VectorXd x = system->GetX();
        
        meas_history.push_back(x + R * VectorXd::Random(num_states));

        VectorXd x_est = ekf->ComputeEstimate(meas_history.back(), B * u);

        VectorXd x_err(1);
        x_err << x_ref(0) - x_est(0);

        VectorXd dxdt_err(1);
        dxdt_err << x_ref(1) - x_est(1);

        u = pid_controller->GenerateControlInput(x_err, dxdt_err);

        t += dt;
        x_history.push_back(x);
        x_ref_history.push_back(x_ref);
        x_est_history.push_back(x_est);
        x_err_history.push_back(x_ref - x);
        t_history.push_back(t);
    }
    // save simulation data for plotting
    std::string directory = "examples";
    std::string problem = system->GetName();
    SaveTimeHistory(directory, problem, t_history, "replace");
    SaveSimDataHistory(directory, problem, "state_history", system->GetColumnNames(), x_history, "replace");
    SaveSimDataHistory(directory, problem, "ref_history", system->GetColumnNames(), x_ref_history, "replace");
    SaveSimDataHistory(directory, problem, "meas_history", system->GetColumnNames(), meas_history, "replace");
    SaveSimDataHistory(directory, problem, "est_history", system->GetColumnNames(), x_est_history, "replace");
    SaveSimDataHistory(directory, problem, "control_history", pid_controller->GetColumnNames(), pid_controller->GetControlInputHistory(), "replace");
    SaveSimDataHistory(directory, problem, "err_history", system->GetColumnNames(), x_err_history, "replace");

    delete ekf;
    delete system;
    return 0;
}