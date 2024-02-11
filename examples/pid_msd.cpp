# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/mass_spring_damper.h"
# include "numerical_solvers/solver_helper_funs.h"
# include "controllers/pid_controller.h"
# include "data_logging/savecsv.h"
# include "data_logging/data_logging_helper_funs.h"

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
    int num_inputs = 2;
    VectorXd x0(num_states);
    x0 << 0.1, 0.0;
    double t0 = 0.0;
    double dh = 0.0001;

    // initialize control input
    VectorXd u(num_inputs);
    std::vector<VectorXd> u_history;

    // initialize mass-spring-damper system
    double k = 1.0;
    double c = 1.0;
    double m = 1.0;
    MatrixXd B_in(num_states, num_inputs);
    B_in << 0, 0,
            1, 1;
    MassSpringDamperSys* system = new MassSpringDamperSys(x0, t0, dh, num_states, B_in, problem, k, c, m);
    std::cout << *system;

    // choose reference trajectory 
    std::string reference_trajectory_type = "step";
    VectorXd x_ref(num_states);

    // choose control action: open_loop or closed_loop
    std::string control_type = "closed_loop";
    
    // set integration duration
    double dt = 0.01; 
    double time_final = 5;

    PID* pid_controller = new PID(num_states);
    if (control_type == "closed_loop") {
        // initialize PID controller
        MatrixXd KP = MatrixXd::Constant(2,2,0.0);
        KP(0,0) = 1.0;
        
        MatrixXd KI = MatrixXd::Constant(2,2,0.0);
        KI(0,0) = 0.01; // ki < (1+kp) // ki < b/m(k+kp)

        MatrixXd KD = MatrixXd::Constant(2,2,0.0);
        KD(1,1) = 4.0;

        // set PID gains
        pid_controller->SetGains(KP, KI, KD);

        std::cout << *pid_controller << "\n";
    } 

     // initialize measured output z
    std::vector<VectorXd> meas_history;
    double measurement_noise = 0.001;
    
    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<double> t_history;
    // system dynamics and controller action
    double t = 0;

    meas_history.push_back(x0);
    x_history.push_back(x0);
    t_history.push_back(t);

    while (t < time_final) {
        int ode_timesteps = dt/dh;
        system->IntegrateODE(ode_timesteps, u);

        VectorXd x = system->GetX();
        meas_history.push_back(x + measurement_noise * VectorXd::Random(2));

        x_ref = CalculateXref(reference_trajectory_type, num_states, t);

        if (control_type == "closed_loop") {
            u = CalculateControlInput(x_ref, x, pid_controller, num_states);
        } else if (control_type == "open_loop") {
            u = CalculateControlInput(x_ref, num_states);
            u_history.push_back(u);
        }

        t += dt;
        x_history.push_back(x);
        t_history.push_back(t);
    }

    // save final outputs to csv files
    SaveTimeHistory(directory, problem, t_history);
    SaveSimDataHistory(directory, problem, "state_history", system->GetColumnNames(), x_history);
    SaveSimDataHistory(directory, problem, "meas_history", system->GetColumnNames(), meas_history);
    
    if (control_type == "closed_loop") {
        SaveSimDataHistory(directory, problem, "control_history", pid_controller->GetColumnNames(), pid_controller->GetControlInputHistory());
        SaveSimDataHistory(directory, problem, "err_history", system->GetColumnNames(), pid_controller->GetErrorHistory());

    } else if (control_type == "open_loop") {
        SaveSimDataHistory(directory, problem, "control_history", system->GetColumnNames(), u_history);
    }

    delete system;
    delete pid_controller;

    return 0;
}