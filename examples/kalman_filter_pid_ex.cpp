# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "system_models/mass_spring_damper.h"
# include "controllers/pid_controller.h"
# include "data_logging/data_logging_helper_funs.h"
# include "state_estimators/kalman_filter.h"
# include "controllers/ackermans_formula_pole_placement.h"
# include "controllers/controller_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    int num_states = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 0.0, 0.0;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 1, 0.0,
          0.0, 1;

    // initialize control input
    int num_inputs = 1;
    VectorXd u(num_inputs);

    // define mass-spring-damper system variables
    double t0 = 0.0;
    double dh = 1e-4;

    // initialize mass-spring-damper system
    double k = 1.0;
    double c = 1.0;
    double m = 1.0;
    MatrixXd B(num_states, num_inputs);
    B << 0, 1;
    MassSpringDamperSys* system = new MassSpringDamperSys(x0, t0, dh, num_states, B, "msd_kf_pid", k, c, m);

    // set integration duration
    double dt = 1e-2; 
    double time_final = 10.0;

    MatrixXd A = system->GetA();

    // define state matrix A for discrete system
    MatrixXd A_state(num_states, num_states);
    A_state = MatrixXd::Identity(num_states, num_states) + A * dt;

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
    VectorXd x_ref(num_states);
    x_ref << 1, 0;

    // coefficients of desired characteristic polynomial
    VectorXd coeffs(num_states+1);
    coeffs << 1, 4, 4;

    VectorXd K = FindKAckermanFormula(A, B, coeffs, "coeffs");

    CheckSysStability(A - B * K.transpose(), "Closed Loop");

    PID* pid_controller = new PID(1);
    // initialize PID controller
    MatrixXd KP(1,1);
    KP(0,0) = K(0);
    
    MatrixXd KI(1,1);
    KI(0,0) = 0.0;

    MatrixXd KD(1,1);
    KD(0,0) = K(1);

    // set PID gains
    pid_controller->SetGains(KP, KI, KD);

    std::cout << *pid_controller << "\n";

    MatrixXd C(1, num_states);    
    C << 1, 0;

    VectorXd N_bar = ScaleCLTransferFunction(A, B, C, K, x_ref(0));
    std::cout << "\n N_bar = " << N_bar << "\n";

    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_ref_history;
    std::vector<VectorXd> x_est_history;
    std::vector<double> t_history;

    // system dynamics and controller action
    double t = 0;

    meas_history.push_back(x0);
    x_history.push_back(x0);
    x_ref_history.push_back(x0);
    x_est_history.push_back(x0);
    t_history.push_back(t);

    while (t < time_final) {
        int ode_timesteps = dt/dh;
        system->IntegrateODE(ode_timesteps, u);

        VectorXd x = system->GetX();

        meas_history.push_back(x + R * VectorXd::Random(num_states));
        
        VectorXd x_est = kf->ComputeEstimate(meas_history.back());

        VectorXd x_err(1);
        x_err << x(0);
        VectorXd dxdt_err(1);
        dxdt_err << x(1);
        // u = N_bar * x_ref - K * x;
        u = N_bar * x_ref(0) - pid_controller->GenerateControlInput(x_err, dxdt_err);

        t += dt;

        x_history.push_back(x);
        x_ref_history.push_back(x_ref);
        x_est_history.push_back(x_est);
        t_history.push_back(t);
    }
    // save simulation data for plotting
    std::string directory = "examples";
    std::string problem = "kf_pid";
    SaveTimeHistory(directory, problem, t_history, "replace");
    SaveSimDataHistory(directory, problem, "state_history", system->GetColumnNames(), x_history, "replace");
    SaveSimDataHistory(directory, problem, "ref_history", system->GetColumnNames(), x_ref_history, "replace");
    SaveSimDataHistory(directory, problem, "meas_history", system->GetColumnNames(), meas_history, "replace");
    SaveSimDataHistory(directory, problem, "est_history", system->GetColumnNames(), x_est_history, "replace");
    SaveSimDataHistory(directory, problem, "control_history", pid_controller->GetColumnNames(), pid_controller->GetControlInputHistory(), "replace");
    SaveSimDataHistory(directory, problem, "err_history", system->GetColumnNames(), pid_controller->GetXErrorHistory(), "replace");


    delete kf;
    delete system;
    delete pid_controller;
    return 0;
}