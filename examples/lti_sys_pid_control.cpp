# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/linear_time_invariant_system.h"
# include "controllers/pid_controller.h"
# include "data_logging/data_logging_helper_funs.h"
# include "controllers/ackermans_formula_pole_placement.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main () {

    std::string soln_directory = "examples";
    std::string problem = "lti";

    // initialize state
    int num_states = 2; 
    int num_inputs = 1;
    VectorXd x0(num_states);
    x0 << 0.1, 0.0;
    double t0 = 0.0;
    double dh = 1e-4;

    // initialize control input
    VectorXd u(num_inputs);
    std::vector<VectorXd> u_history;

    // initialize LTI system
    MatrixXd A(num_states, num_states);
    A << 0, 0,
         1, 1;
    MatrixXd B(num_states, num_inputs);
    B << 1, 0;
    LinearTimeInvariantSys* system = new LinearTimeInvariantSys(x0, t0, dh, 
                                                                num_states, problem, A, B);
    std::cout << *system;

    // choose reference trajectory 
    VectorXd x_ref(num_states);
    x_ref << 1.0, 0.0;

    // choose controller gains using Ackermans formula
    VectorXd desired_roots(num_states);
    desired_roots << -6, -5;

    AckermansFormulaPolePlacement* pole_placement = new AckermansFormulaPolePlacement("roots", desired_roots, A, B);
    VectorXd K = pole_placement->AckermansFormula();
    std::cout << "\nController gains: \n" << K;

    // set integration duration
    double dt = 1e-2; 
    double time_final = 5;

    PID* pid_controller = new PID(num_states);
    // initialize PID controller
    MatrixXd KP = MatrixXd::Constant(2,2,0.0);
    KP(0,0) = K(0);
    
    MatrixXd KI = MatrixXd::Constant(2,2,0.0);
    KI(0,0) = 0.01; // ki < (1+kp) // ki < b/m(k+kp)

    MatrixXd KD = MatrixXd::Constant(2,2,0.0);
    KD(0,0) = K(1);

    // set PID gains
    pid_controller->SetGains(KP, KI, KD);

    std::cout << *pid_controller << "\n";

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

    int ode_timesteps = dt/dh;

    while (t < time_final) {
        system->IntegrateODE(ode_timesteps, u);

        VectorXd x = system->GetX();
        meas_history.push_back(x + measurement_noise * VectorXd::Random(2));

        pid_controller->CalculateError(x_ref, x);

        VectorXd uk = pid_controller->GenerateControlInput();
        VectorXd u(num_inputs);
        u << uk(0);

        t += dt;
        x_history.push_back(x);
        t_history.push_back(t);
    }

    // save final outputs to csv files
    SaveTimeHistory(soln_directory, problem, t_history);
    SaveSimDataHistory(soln_directory, problem, "state_history", system->GetColumnNames(), x_history);
    SaveSimDataHistory(soln_directory, problem, "meas_history", system->GetColumnNames(), meas_history);
    SaveSimDataHistory(soln_directory, problem, "control_history", pid_controller->GetColumnNames(), pid_controller->GetControlInputHistory());
    SaveSimDataHistory(soln_directory, problem, "err_history", system->GetColumnNames(), pid_controller->GetErrorHistory());

    delete system;
    delete pid_controller;
    delete pole_placement;
    return 0;
}