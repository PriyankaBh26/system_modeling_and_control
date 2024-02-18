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

void CheckOLSysStability(MatrixXd A) {
    // Compute the eigenvalues of A
    Eigen::EigenSolver<MatrixXd> solver1(A);
    Eigen::VectorXcd evals = solver1.eigenvalues();
    std::cout << "A eigenvalues:\n"  << evals.transpose();
    int stable_eval = 0;
    for (const auto& eval : evals) {
        if (eval.real() > 0) {
            std::cout << "\n Open Loop System is unstable \n";
            break;
        } else if (eval.real() == 0) {
            std::cout << "\n Open Loop System is marginally stable \n";
            break;
        } else {
            stable_eval += 1;
        }
    }
    if (stable_eval == evals.size()) {
        std::cout << "\n Open Loop System is stable \n";
    }
}

VectorXd FindK(MatrixXd A, MatrixXd B, VectorXd coeffs) {
    AckermansFormulaPolePlacement* system = new AckermansFormulaPolePlacement("coeffs", coeffs, A, B);
    VectorXd K = system->AckermansFormula();
    delete system;
    return K;
}

VectorXd ScaleCLTransferFunction(MatrixXd A, MatrixXd B, MatrixXd C, VectorXd K, double r_ss) {
    int num_states = A.rows();
    int num_op = C.rows();

    MatrixXd AR(num_states+num_op,num_states+num_op);
    AR.block(0,0,num_states,num_states) = A;
    AR.block(0,num_states,num_states,1) = B;
    AR.block(num_states,0,num_op,num_states) = C;

    VectorXd b = VectorXd::Zero(num_states+num_op);
    b(num_states+num_op-1) = r_ss;

    VectorXd xu = AR.colPivHouseholderQr().solve(b);
    VectorXd Nx = xu.head(2).array() / r_ss;
    VectorXd Nu = xu.tail(1) / r_ss;

    VectorXd N_bar = Nu + K.transpose() * Nx;
    // u = N_bar * r - K * x;
    return N_bar;
}

int main () {

    std::string soln_directory = "examples";
    std::string problem = "lti";

    // initialize state
    int num_states = 2; 
    VectorXd x0(num_states);
    x0 << 0.0, 0.0;
    double t0 = 0.0;
    double dh = 1e-4;

    // initialize LTI system
    MatrixXd A(num_states, num_states);
    A << 1, 1,
         1, 2;

    CheckOLSysStability(A);

    MatrixXd B(num_states, 1);    
    B << 1, 0;
    
    MatrixXd C(1, num_states);    
    C << 1, 0;

    LinearTimeInvariantSys* system = new LinearTimeInvariantSys(x0, t0, dh, 
                                                                num_states, problem, A, B);
    std::cout << *system;

    // choose reference trajectory 
    VectorXd x_ref(num_states);
    x_ref << 1.0, 0.0;

    // coefficients of desired characteristic polynomial
    VectorXd coeffs(num_states+1);
    coeffs << 1, 11, 30;

    VectorXd K = FindK(A, B, coeffs);

    PID* pid_controller = new PID(1);
    // initialize PID controller
    MatrixXd KP(1,1);
    KP(0,0) = K(0);
    
    MatrixXd KI(1,1);
    KI(0,0) = 0.0; // ki < (1+kp) // ki < b/m(k+kp)

    MatrixXd KD(1,1);
    KD(0,0) = K(1);

    // set PID gains
    pid_controller->SetGains(KP, KI, KD);

    std::cout << *pid_controller << "\n";

    VectorXd N_bar = ScaleCLTransferFunction(A, B, C, K, x_ref(0));
    std::cout << "\n N_bar = " << N_bar << "\n";

    // set integration duration
    double dt = 1e-2; 
    double time_final = 10;

     // initialize measured output z
    std::vector<VectorXd> meas_history;
    double measurement_noise = 0.01;

    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<double> t_history;
    std::vector<VectorXd> x_ref_history;
    // system dynamics and controller action
    double t = 0;

    meas_history.push_back(x0);
    x_history.push_back(x0);
    x_ref_history.push_back(x_ref);
    t_history.push_back(t);

    int ode_timesteps = dt/dh;

    // initialize control input
    VectorXd u(1);
    std::vector<VectorXd> u_history;

    while (t < time_final) {
        system->IntegrateODE(ode_timesteps, u);
        VectorXd x = system->GetX();

        VectorXd z = x + measurement_noise * VectorXd::Random(num_states);
        meas_history.push_back(z);

        VectorXd x_err(1);
        x_err << z(0);
        VectorXd dxdt_err(1);
        dxdt_err << z(1);

        u = N_bar * x_ref(0) - pid_controller->GenerateControlInput(x_err, dxdt_err);

        t += dt;
        x_history.push_back(x);
        x_ref_history.push_back(x_ref);
        t_history.push_back(t);
    }

    // save final outputs to csv files
    SaveTimeHistory(soln_directory, problem, t_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "state_history", system->GetColumnNames(), x_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "meas_history", system->GetColumnNames(), meas_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "ref_history", system->GetColumnNames(), x_ref_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "control_history", pid_controller->GetColumnNames(), pid_controller->GetControlInputHistory(), "replace");
    SaveSimDataHistory(soln_directory, problem, "err_history", system->GetColumnNames(), pid_controller->GetXErrorHistory(), "replace");

    delete system;
    delete pid_controller;

    return 0;
}