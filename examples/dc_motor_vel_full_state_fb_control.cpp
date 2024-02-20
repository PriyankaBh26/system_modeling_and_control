# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "system_models/dc_motor_velocity.h"
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
        x_ref << A;
    } else if (reference_trajectory_type == "sine") {
        double A = 1.0;
        double f = 1;
        x_ref << A*sin(2*M_PI*f*t);
    } else if (reference_trajectory_type == "ramp") {
        double A = 1.0;
        double ramp_slope = 1.0;
        double max_ramp_time = 10;
        if (t < max_ramp_time) {
            x_ref << A*ramp_slope*t;
        } else {
            x_ref << ramp_slope*max_ramp_time;

        }
    }
    return x_ref;
}

int main () {

    // initialize mass-spring-damper system
    int num_states = 2; 
    VectorXd x0(num_states);
    x0 << 0.0, 0.0;
    double t0 = 0.0;
    double dh = 0.0001;
    
    // initialize dc motor velocity model
    // reference: https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
    double J = 0.01; // kg.m^2
    double b = 0.1; // N.m.s
    double Kd = 0.01; // V/rad/s
    double R = 1; // ohm
    double L = 0.5; // H

    DCMotorVelocity* system = new DCMotorVelocity(x0, t0, dh,
                                            num_states, "dc_motor_vel", 
                                            J, b, Kd, R, L);

    // choose reference trajectory 
    std::string reference_trajectory_type = "step";
    VectorXd x_ref(num_states);
    x_ref << 1, 0;

    // choose control action: open_loop or closed_loop
    std::string control_type = "closed_loop";
    // initialize control input
    VectorXd u(1);
    std::vector<VectorXd> u_history;
    
    // set integration duration
    double dt = 0.01; 
    double time_final = 5;

    PID* pid_controller = new PID(1);
        
    // coefficients of desired characteristic polynomial
    VectorXd coeffs(num_states+1);
    coeffs << 1, 4, 4;

    VectorXd K = FindKAckermanFormula(system->GetA(), system->GetB(), coeffs, "coeffs");

    // initialize PID controller
    MatrixXd KP = MatrixXd::Constant(1,1,0.0);
    KP(0,0) = K(0);
    
    MatrixXd KI = MatrixXd::Constant(1,1,0.0);
    KI(0,0) = 0.0; // ki < (1+kp) // ki < b/m(k+kp)

    MatrixXd KD = MatrixXd::Constant(1,1,0.0);
    KD(0,0) = K(1);

    // set PID gains
    pid_controller->SetGains(KP, KI, KD);

    std::cout << *pid_controller << "\n";

    MatrixXd C(1, num_states);    
    C << 1, 0;
    
    VectorXd N_bar = ScaleCLTransferFunction(system->GetA(), system->GetB(), C, K, x_ref(0));
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

    while (t < time_final) {
        int ode_timesteps = dt/dh;
        system->IntegrateODE(ode_timesteps, u);

        VectorXd x = system->GetX();
        VectorXd z = x + measurement_noise * VectorXd::Random(num_states);
        meas_history.push_back(z);

        x_ref = CalculateXref(reference_trajectory_type, 1, t);

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
    std::string directory = "examples";
    std::string problem = "pid_dc_motor";
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