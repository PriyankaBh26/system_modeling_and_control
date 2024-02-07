# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "data_logging/data_logging_helper_funs.h"
# include "numerical_solvers/rk_ode_solver.h"
# include "controllers/pidcontroller.h"
# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void SaveSimulationData(OdeSolver* ode, 
                        PID* pid_controller, 
                        std::vector<VectorXd>& x_history, 
                        std::vector<double> t_history) {

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


void SaveSimulationData(OdeSolver* ode, 
                        std::vector<VectorXd>& x_history, 
                        std::vector<double> t_history, 
                        std::vector<VectorXd>& u_history) {

    // save final output x to csv file
    std::string filename = "examples/" + ode->GetName() + "_solution.csv";
    std::vector<std::string> columnNames = ode->GetColumnNames();
    WriteMatToFile(filename, columnNames, x_history);
    std::cout << x_history.size() << " " << x_history[0].size();

    // save control input history to csv file
    filename = "examples/" + ode->GetName() + "control_ip_history.csv";
    columnNames = ode->GetControlInputColumnNames(u_history[0]);
    WriteMatToFile(filename, columnNames, u_history);

    // save time to csv file
    filename = "examples/" + ode->GetName() + "_time.csv";
    columnNames = {"time"};
    WriteVecToFile(filename, columnNames, t_history);
}

void SaveKFSimulationData(OdeSolver* ode, 
                          std::vector<VectorXd>& x_history, 
                          std::vector<double> t_history, 
                          std::vector<VectorXd>& x_est_history, 
                          std::vector<VectorXd>& z_history) {

    // save final output x to csv file
    std::string filename = "examples/" + ode->GetName() + "_solution.csv";
    std::vector<std::string> column_names = ode->GetColumnNames();
    WriteMatToFile(filename, column_names, x_history);
    std::cout << x_history.size() << " " << x_history[0].size();

    // save estimated state history to csv file
    filename = "examples/" + ode->GetName() + "_meas_history.csv";
    WriteMatToFile(filename, column_names, z_history);

    // save estimated state history to csv file
    filename = "examples/" + ode->GetName() + "_est_history.csv";
    WriteMatToFile(filename, column_names, x_est_history);

    // save time to csv file
    filename = "examples/" + ode->GetName() + "_time.csv";
    column_names = {"time"};
    WriteVecToFile(filename, column_names, t_history);
}


void SaveKFPIDSimulationData(OdeSolver* ode, 
                        std::vector<VectorXd>& x_history, 
                        std::vector<double> t_history, 
                        std::vector<VectorXd>& x_est_history, 
                        std::vector<VectorXd>& z_history,
                        std::vector<VectorXd>& u_history) {

    // save final output x to csv file
    std::string filename = "examples/" + ode->GetName() + "_solution.csv";
    std::vector<std::string> column_names = ode->GetColumnNames();
    WriteMatToFile(filename, column_names, x_history);
    std::cout << x_history.size() << " " << x_history[0].size();

    // save control history to csv file
    filename = "examples/" + ode->GetName() + "_control_history.csv";
    std::vector<std::string> u_column_names = {"U1", "U2"};
    WriteMatToFile(filename, u_column_names, u_history);

    // save measured state history to csv file
    filename = "examples/" + ode->GetName() + "_meas_history.csv";
    WriteMatToFile(filename, column_names, z_history);

    // save estimated state history to csv file
    filename = "examples/" + ode->GetName() + "_est_history.csv";
    WriteMatToFile(filename, column_names, x_est_history);

    // save time to csv file
    filename = "examples/" + ode->GetName() + "_time.csv";
    column_names = {"time"};
    WriteVecToFile(filename, column_names, t_history);
}