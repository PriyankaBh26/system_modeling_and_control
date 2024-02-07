# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "numerical_solvers/solver_helper_funs.h"
# include "controllers/pidcontroller.h"
# include "data_logging/savecsv.h"
# include "data_logging/data_logging_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;

class solve_mass_spring_damper : public OdeSolver {
    public: 

        solve_mass_spring_damper(VectorXd y0, double t0, double dt0) : OdeSolver(y0, t0, dt0) {};

        VectorXd f(double t, VectorXd y, VectorXd u) override {
            double k = 1.0;
            double c = 1.0;
            double m = 1.0;

            MatrixXd A(2,2);
            A(0,0) = 0;
            A(0,1) = 1;
            A(1,0) = -k/m;
            A(1,1) = -c/m;

            MatrixXd B(2,2);
            B(0,0) = 0;
            B(0,1) = 0;
            B(1,0) = 1/m;
            B(1,1) = 1/m;

            VectorXd yd = A * y + B * u;
            return yd;
        }

        std::string GetName() override {
            return "mass_spring_damper";
        }

        std::vector<std::string> GetColumnNames() override {
            return {"Pos", "Vel"};
        }

};

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

    // initialize mass-spring-damper system
    int num_states = 2; 
    VectorXd y0(num_states);
    y0 << 0.1, 0.0;
    double t0 = 0.0;
    double dh = 0.0001;

    OdeSolver* ode = new solve_mass_spring_damper(y0, t0, dh);

    // choose reference trajectory 
    std::string reference_trajectory_type = "step";
    VectorXd x_ref(num_states);

    // choose control action: open_loop or closed_loop
    std::string control_type = "open_loop";
    // initialize control input
    VectorXd u(num_states);
    std::vector<VectorXd> u_history;
    
    // set integration duration
    double dt = 0.01; 
    double time_final = 1;

    PID* pid_controller = new PID(num_states);
    if (control_type == "closed_loop") {
        // initialize PID controller
        MatrixXd KP = MatrixXd::Constant(2,2,0.0);
        KP(0,0) = 1.0;
        
        MatrixXd KI = MatrixXd::Constant(2,2,0.0);
        KI(0,0) = 0.01; // ki < (1+kp) // ki < b/m(k+kp)

        MatrixXd KD = MatrixXd::Constant(2,2,0.0);
        KD(0,0) = 4.0;

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