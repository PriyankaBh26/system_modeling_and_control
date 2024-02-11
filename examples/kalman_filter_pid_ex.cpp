# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/mass_spring_damper.h"
# include "controllers/pid_controller.h"
# include "data_logging/savecsv.h"
# include "data_logging/data_logging_helper_funs.h"
# include "state_estimators/kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

VectorXd CalculateXref(std::string reference_trajectory_type, int num_states, double t) {
    VectorXd x_ref(num_states);
    if (reference_trajectory_type == "step") {
        double A = 1.0;
        x_ref << A, 0.0;
    } else if (reference_trajectory_type == "sine") {
        double A = 1.0;
        double f = 1;
        x_ref << A*sin(2*M_PI*f*t), A*2*M_PI*f*cos(2*M_PI*f*t);
    } else if (reference_trajectory_type == "ramp") {
        double A = 1.0;
        double ramp_slope = 1.0;
        double max_ramp_time = 10;
        if (t < max_ramp_time) {
            x_ref << A*ramp_slope*t, A*ramp_slope;
        } else {
            x_ref << ramp_slope*max_ramp_time, 0;

        }
    }
    return x_ref;
}

int main() {
    int num_states = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 0.0, 0.0;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 10, 0.0,
          0.0, 10;

    // initialize control input
    int num_inputs = 2;
    VectorXd u(num_inputs);

    // define mass-spring-damper system variables
    double t0 = 0.0;
    double dh = 1e-4;

    // initialize mass-spring-damper system
    double k = 1.0;
    double c = 1.0;
    double m = 1.0;
    MatrixXd B_in(num_states, num_inputs);
    B_in << 0, 0,
            1, 1;
    MassSpringDamperSys* system = new MassSpringDamperSys(x0, t0, dh, num_states, B_in, "msd_kf_pid", k, c, m);

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
         0.0, 0.0;
    // define measurement noise matrix
    MatrixXd R(num_states, num_states);
    R << 1e-2, 0.0,
          0.0, 1e-2;

    // initialize kalman filter object
    VectorXd x0_m(num_states);
    x0_m << 0.0, 0.0;
    KalmanFilter* kf = new KalmanFilter(x0_m, P0, A_state, Q, H, R);

    // initialize measurement z
    std::vector<VectorXd> meas_history;

    // choose reference trajectory 
    std::string reference_trajectory_type = "step";
    VectorXd x_ref(num_states);

    // initialize PID controller
    PID* pid_controller = new PID(num_states);
    MatrixXd KP = MatrixXd::Constant(2,2,0.0);
    KP(0,0) = 0.8;
    KP(1,1) = 4.0;
    
    MatrixXd KI = MatrixXd::Constant(2,2,0.0);
    KI(0,0) = 0.05; // ki < (1+kp) // ki < b/m(k+kp)

    MatrixXd KD = MatrixXd::Constant(2,2,0.0);
    KD(0,0) = 0.0;

    // set PID gains
    pid_controller->SetGains(KP, KI, KD);

    std::cout << *pid_controller << "\n";

    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_est_history;
    std::vector<double> t_history;

    // system dynamics and controller action
    double t = 0;

    meas_history.push_back(x0);
    x_history.push_back(x0);
    x_est_history.push_back(x0);
    t_history.push_back(t);

    while (t < time_final) {
        int ode_timesteps = dt/dh;
        system->IntegrateODE(ode_timesteps, u);

        VectorXd x = system->GetX();

        meas_history.push_back(x + R * VectorXd::Random(num_states));
        
        x_ref = CalculateXref(reference_trajectory_type, num_states, t);

        VectorXd x_est = kf->ComputeEstimate(meas_history.back());

        pid_controller->CalculateError(x_ref, x_est);
        u = pid_controller->GenerateControlInput();

        t += dt;

        x_history.push_back(x);
        x_est_history.push_back(x_est);
        t_history.push_back(t);
    }
    // save simulation data for plotting
    std::string directory = "examples";
    std::string problem = "kf_pid";
    SaveTimeHistory(directory, problem, t_history);
    SaveSimDataHistory(directory, problem, "state_history", system->GetColumnNames(), x_history);
    SaveSimDataHistory(directory, problem, "meas_history", system->GetColumnNames(), meas_history);
    SaveSimDataHistory(directory, problem, "est_history", system->GetColumnNames(), x_est_history);
    SaveSimDataHistory(directory, problem, "control_history", pid_controller->GetColumnNames(), pid_controller->GetControlInputHistory());
    SaveSimDataHistory(directory, problem, "err_history", system->GetColumnNames(), pid_controller->GetErrorHistory());


    delete kf;
    delete system;
    delete pid_controller;
    return 0;
}