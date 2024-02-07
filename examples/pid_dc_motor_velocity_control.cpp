# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/dc_motor_velocity.h"
# include "controllers/pid_controller.h"
# include "data_logging/savecsv.h"
# include "data_logging/data_logging_helper_funs.h"

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
    VectorXd y0(num_states);
    y0 << 0.0, 0.0;
    double t0 = 0.0;
    double dh = 0.0001;
    
    // initialize dc motor velocity model
    // reference: https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
    double J = 0.01; // kg.m^2
    double b = 0.1; // N.m.s
    double K = 0.01; // V/rad/s
    double R = 1; // ohm
    double L = 0.5; // H

    DCMotorVelocity* ode = new DCMotorVelocity(y0, t0, dh,
                                            num_states, "dc_motor_vel", 
                                            J, b, K, R, L);

    // choose reference trajectory 
    std::string reference_trajectory_type = "step";
    VectorXd x_ref(num_states);

    // choose control action: open_loop or closed_loop
    std::string control_type = "open_loop";
    // initialize control input
    VectorXd u(1);
    std::vector<VectorXd> u_history;
    
    // set integration duration
    double dt = 0.01; 
    double time_final = 1;

    PID* pid_controller = new PID(1);
    if (control_type == "closed_loop") {
        // initialize PID controller
        MatrixXd KP = MatrixXd::Constant(1,1,0.0);
        KP(0,0) = 11.0;
        
        MatrixXd KI = MatrixXd::Constant(1,1,0.0);
        KI(0,0) = 0.6; // ki < (1+kp) // ki < b/m(k+kp)

        MatrixXd KD = MatrixXd::Constant(1,1,0.0);
        KD(0,0) = 0.05;

        // set PID gains
        pid_controller->SetGains(KP, KI, KD);

        std::cout << *pid_controller << "\n";
    } 

    // initialize measured output z
    std::vector<VectorXd> z;
    double measurement_noise = 0.001;
    
    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<double> t_history;

    // system dynamics and controller action
    double t = 0;
    while (t < time_final) {
        int ode_timesteps = dt/dh;
        ode->IntegrateODE(ode_timesteps, u);

        VectorXd x = ode->GetX();
        z.push_back(x + measurement_noise * VectorXd::Random(2));

        x_ref = CalculateXref(reference_trajectory_type, 1, t);

        if (control_type == "closed_loop") {
            VectorXd x_in(1);
            x_in << x(0);
            pid_controller->CalculateError(x_ref, x_in);
            u = pid_controller->GenerateControlInput();
        } else if (control_type == "open_loop") {
            double A = 10;
            u = A * x_ref;
            u_history.push_back(u);
        }

        t += dt;
        x_history.push_back(x);
        t_history.push_back(t);
    }

    // save final outputs to csv files
    if (control_type == "closed_loop") {
        SaveSimulationData(ode, pid_controller, x_history, t_history);
        delete pid_controller;
    } else if (control_type == "open_loop") {
        SaveSimulationData(ode, x_history, t_history, u_history);
    }

    delete ode;
    delete pid_controller;
    return 0;
}