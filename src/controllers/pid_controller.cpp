# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>
# include "controllers/pid_controller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

PID::PID(int n) : num_states(n), kp(n,n), ki(n,n), kd(n,n) {
    integral_err_history.push_back(VectorXd::Zero(n));
    x_err_history.push_back(VectorXd::Zero(n));
    dxdt_err_history.push_back(VectorXd::Zero(n));    
    control_input_history.push_back(VectorXd::Zero(n));
    };

void PID::SetGains(const MatrixXd& KP, const MatrixXd& KI, const MatrixXd& KD) {
    kp = KP;
    ki = KI;
    kd = KD;
}

std::vector<VectorXd> PID::GetXErrorHistory() {
    return x_err_history;
}

std::vector<VectorXd> PID::GetDxdtErrorHistory() {
    return dxdt_err_history;
}

std::vector<VectorXd> PID::GetIntegralErrorHistory() {
    return integral_err_history;
}

std::vector<VectorXd> PID::GetControlInputHistory() {
    return control_input_history;
}

VectorXd PID::GenerateControlInput(VectorXd x_err, VectorXd dxdt_err) {
    VectorXd sum = integral_err_history.back() + x_err;
    VectorXd u = kp * x_err + ki * sum + kd * dxdt_err;
    // save control input history
    control_input_history.push_back(u);
    // save error history
    x_err_history.push_back(x_err);
    dxdt_err_history.push_back(dxdt_err);
    integral_err_history.push_back(sum);
    return u;
}

std::vector<std::string> PID::GetColumnNames() {
    std::vector<std::string> column_names;
    for (int i{0}; i<num_states; i++) {
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

