# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "system_models/mass_spring_damper.h"
# include "controllers/pid_controller.h"
# include "data_logging/data_logging_helper_funs.h"
# include "controllers/ackermans_formula_pole_placement.h"
# include "controllers/controller_helper_funs.h"

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

int main () {
    std::string directory = "examples";
    std::string problem = "msd";

    // initialize state
    int num_states = 2; 
    VectorXd x0(num_states);
    x0 << 0.1, 0.0;
    double t0 = 0.0;
    double dh = 0.0001;

    // initialize mass-spring-damper system
    double k = 1.0;
    double c = 1.0;
    double m = 1.0;
    MatrixXd B_in(num_states, 1);
    B_in << 0, 1;
    MassSpringDamperSys* system = new MassSpringDamperSys(x0, t0, dh, num_states, B_in, problem, k, c, m);
    std::cout << *system;

    // choose reference trajectory 
    std::string reference_trajectory_type = "step";
    VectorXd x_ref(num_states);
    x_ref << 1, 0;

    // choose control action: open_loop or closed_loop
    std::string control_type = "closed_loop";
    
    // set integration duration
    double dt = 0.01; 
    double time_final = 5;

    // coefficients of desired characteristic polynomial
    VectorXd coeffs(num_states+1);
    coeffs << 1, 4, 4;

    MatrixXd A = system->GetA();
    MatrixXd B = system->GetB();
    MatrixXd C(1, num_states);    
    C << 1, 0;

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

    VectorXd N_bar = ScaleCLTransferFunction(A, B, C, K, x_ref(0));
    std::cout << "\n N_bar = " << N_bar << "\n";

     // initialize measured output z
    std::vector<VectorXd> meas_history;
    double measurement_noise = 0.001;
    
    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_ref_history;
    std::vector<double> t_history;
    // system dynamics and controller action
    double t = 0;

    meas_history.push_back(x0);
    x_history.push_back(x0);
    x_ref_history.push_back(x_ref);
    t_history.push_back(t);


    // initialize control input
    VectorXd u(1);
    std::vector<VectorXd> u_history;

    while (t < time_final) {
        int ode_timesteps = dt/dh;
        system->IntegrateODE(ode_timesteps, u);


        VectorXd x = system->GetX();
        VectorXd z = x + measurement_noise * VectorXd::Random(num_states);
        meas_history.push_back(z);

        x_ref = CalculateXref(reference_trajectory_type, num_states, t);

        VectorXd x_err(1);
        x_err << z(0);
        VectorXd dxdt_err(1);
        dxdt_err << z(1);
        // u = N_bar * x_ref - K * x;
        u = N_bar * x_ref(0) - pid_controller->GenerateControlInput(x_err, dxdt_err);

        t += dt;
        x_history.push_back(x);
        x_ref_history.push_back(x_ref);
        t_history.push_back(t);
    }

    // save final outputs to csv files
    SaveTimeHistory(directory, problem, t_history, "replace");
    SaveSimDataHistory(directory, problem, "state_history", system->GetColumnNames(), x_history, "replace");
    SaveSimDataHistory(directory, problem, "ref_history", system->GetColumnNames(), x_ref_history, "replace");
    SaveSimDataHistory(directory, problem, "meas_history", system->GetColumnNames(), meas_history, "replace");
    
    SaveSimDataHistory(directory, problem, "control_history", pid_controller->GetColumnNames(), pid_controller->GetControlInputHistory(), "replace");
    SaveSimDataHistory(directory, problem, "err_history", system->GetColumnNames(), pid_controller->GetXErrorHistory(), "replace");

    delete system;
    delete pid_controller;

    return 0;
}