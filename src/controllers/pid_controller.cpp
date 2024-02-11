# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>
# include "controllers/pid_controller.h"

PID::PID(int n) : kp(n,n), ki(n,n), kd(n,n), error(n), d_error(n), sum(n), previous_error(n) {
    error_history.push_back(previous_error);
    control_input_history.push_back(previous_error);
    };

void PID::SetGains(const Eigen::MatrixXd& KP, const Eigen::MatrixXd& KI, const Eigen::MatrixXd& KD) {
    kp = KP;
    ki = KI;
    kd = KD;
}

void PID::CalculateError(Eigen::VectorXd x_ref, Eigen::VectorXd x) {
    error = x_ref - x;
    d_error = error - previous_error;
    sum += error; 
    previous_error = error;
    // save error history
    error_history.push_back(error);
}

std::vector<Eigen::VectorXd> PID::GetErrorHistory() {
    return error_history;
}

std::vector<Eigen::VectorXd> PID::GetControlInputHistory() {
    return control_input_history;
}

Eigen::VectorXd PID::GenerateControlInput() {
    Eigen::VectorXd u = kp * error + ki * sum + kd * d_error;
    // save control input history
    control_input_history.push_back(u);
    return u;
}

std::vector<std::string> PID::GetColumnNames() {
    std::vector<std::string> column_names;
    int ip_size = error.size();
    for (int i{0}; i<ip_size; i++) {
        column_names.push_back("U" + std::to_string(i));
    }
    return column_names;}

std::ostream& operator << (std::ostream& out, const PID& PID) {
    out << "\nPrinting PID controller gains:\n";
    out << "KP = \n" << PID.kp << "\n";
    out << "KI = \n" << PID.ki << "\n";
    out << "KD = \n" << PID.kd << "\n";
    return out;
}

