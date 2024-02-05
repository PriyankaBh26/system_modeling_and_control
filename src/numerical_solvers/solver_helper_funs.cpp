# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "numerical_solvers/solver_helper_funs.h"
# include "controllers/pidcontroller.h"
# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


VectorXd CalculateControlInput(VectorXd& x_ref, VectorXd& x, PID* pid_controller, int num_states) {
    VectorXd u(num_states);
    pid_controller->CalculateError(x_ref, x);
    u = pid_controller->GenerateControlInput();
    return u;
}

VectorXd CalculateControlInput(VectorXd& x_ref, int num_states) {
    VectorXd u(num_states);
    double A = 10;
    u = A * x_ref;
    return u;
}

void SaveSimulationData(OdeSolver* ode, PID* pid_controller, std::vector<VectorXd>& x_history, std::vector<double> t_history) {

    // save final output x to csv file
    std::string filename = "examples/" + ode->GetName() + "_solution.csv";
    std::vector<std::string> columnNames = ode->GetColumnNames();
    WriteMatToFile(filename, columnNames, x_history);
    std::cout << x_history.size() << " " << x_history[0].size();

    // save final error history to csv file
    std::vector<VectorXd> error_history = pid_controller->GetErrorHistory();
    filename = "examples/" + ode->GetName() + "err_history.csv";
    WriteMatToFile(filename, columnNames, error_history);

    // save control input history to csv file
    std::vector<VectorXd> control_input_history = pid_controller->GetControlInputHistory();
    filename = "examples/" + ode->GetName() + "control_ip_history.csv";
    columnNames = pid_controller->GetColumnNames();
    WriteMatToFile(filename, columnNames, control_input_history);

    // save time to csv file
    filename = "examples/" + ode->GetName() + "_time.csv";
    columnNames = {"time"};
    WriteVecToFile(filename, columnNames, t_history);
}


void SaveSimulationData(OdeSolver* ode, std::vector<VectorXd>& x_history, std::vector<double> t_history, std::vector<VectorXd>& u_history) {

    // save final output x to csv file
    std::string filename = "examples/" + ode->GetName() + "_solution.csv";
    std::vector<std::string> columnNames = ode->GetColumnNames();
    WriteMatToFile(filename, columnNames, x_history);
    std::cout << x_history.size() << " " << x_history[0].size();

    // save control input history to csv file
    filename = "examples/" + ode->GetName() + "control_ip_history.csv";
    columnNames = ode->GetControlInputColumnNames();
    WriteMatToFile(filename, columnNames, u_history);

    // save time to csv file
    filename = "examples/" + ode->GetName() + "_time.csv";
    columnNames = {"time"};
    WriteVecToFile(filename, columnNames, t_history);
}
