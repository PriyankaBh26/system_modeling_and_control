# include <iostream>
# include <tuple>
# include <Eigen/Dense>

# include "serial_chain_robots/serial_chain_robot_helper_funs.h"
# include "serial_chain_robots/serial_chain_robot_dynamics.h"
# include "serial_chain_robots/serial_chain_robot_pid_control.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

SerialChainRobotPIDControl::SerialChainRobotPIDControl(int n,
                                                       VectorXd g_vec, 
                                        std::vector<MatrixXd> Mlist,
                                        std::vector<MatrixXd> Glist, 
                                              MatrixXd screw_space) : SerialChainRobotDynamics(n, g_vec, Mlist, Glist, screw_space), 
                                                                        num_joints(n), Kp(n), Ki(n), Kd(n), g(g_vec) {
        VectorXd zero_vec = VectorXd::Zero(num_joints);
        q_error_history.push_back(zero_vec);
        sum_error_history.push_back(zero_vec);
        dq_error_history.push_back(zero_vec);
        control_history.push_back(zero_vec);
        q_history.push_back(zero_vec);
        dq_history.push_back(zero_vec);
        d2q_history.push_back(zero_vec);
        tau_history.push_back(zero_vec);
        };

void SerialChainRobotPIDControl::SetPIDGains(VectorXd Kp_in, VectorXd Ki_in, VectorXd Kd_in) {
    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;
};

VectorXd SerialChainRobotPIDControl::ControlInput(VectorXd q_ref, VectorXd dq_ref, 
                                                  VectorXd q, VectorXd dq) {
    VectorXd e = q_ref - q;
    VectorXd sum_e = sum_error_history.back() + e;
    VectorXd de = dq_ref - dq;
    VectorXd u = Kp.array() * e.array() + Ki.array() * sum_e.array() + Kd.array() * de.array();

    q_error_history.push_back(e);
    sum_error_history.push_back(sum_e);
    dq_error_history.push_back(de);
    control_history.push_back(u);

    return u;
};

VectorXd SerialChainRobotPIDControl::ComputeTorque(VectorXd q_ref, VectorXd dq_ref, VectorXd d2q_ref, 
                                                    VectorXd q, VectorXd dq) {
    VectorXd zeroF = VectorXd::Zero(6);
    VectorXd tau = MassMatrix(q) * SerialChainRobotPIDControl::ControlInput(q_ref, dq_ref, q, dq) \
                    + InverseDynamics(q, dq, d2q_ref, g, zeroF);

    return tau;
};

std::tuple<VectorXd, VectorXd, VectorXd, VectorXd> SerialChainRobotPIDControl::ComputedTorqueControl(VectorXd q_ref, VectorXd dq_ref, VectorXd d2q_ref, 
                                                                                                    VectorXd q, VectorXd dq, VectorXd Ftip, 
                                                                                                    double dt, double dh) {
    VectorXd tau = SerialChainRobotPIDControl::ComputeTorque(q_ref, dq_ref, d2q_ref, q, dq);
    int num_steps = dt / dh;
    VectorXd d2q(num_joints);
    for (int i{0}; i<num_steps; i++) {
        d2q = ForwardDynamics(q, dq, Ftip, tau);
        q = EulerStepUpdate(q, dq, dh);
        dq = EulerStepUpdate(dq, d2q, dh);
    }
    q_history.push_back(q);
    dq_history.push_back(dq);
    d2q_history.push_back(d2q);
    tau_history.push_back(tau);
    return std::make_tuple(q, dq, d2q, tau);
};

std::vector<VectorXd> SerialChainRobotPIDControl::GetQErrorHistory() {return q_error_history;};

std::vector<VectorXd> SerialChainRobotPIDControl::GetSumErrorHistory() {return sum_error_history;};

std::vector<VectorXd> SerialChainRobotPIDControl::GetDqErrorHistory() {return dq_error_history;};

std::vector<VectorXd> SerialChainRobotPIDControl::GetControlHistory() {return control_history;};

std::vector<VectorXd> SerialChainRobotPIDControl::GetQHistory() {return q_history;};

std::vector<VectorXd> SerialChainRobotPIDControl::GetDqHistory() {return dq_history;};

std::vector<VectorXd> SerialChainRobotPIDControl::GetD2qHistory() {return d2q_history;};

std::vector<VectorXd> SerialChainRobotPIDControl::GetTauHistory() {return tau_history;};


std::ostream& operator << (std::ostream& out, SerialChainRobotPIDControl& robot) {
    out << "\n Serial Chain robot PID control: \n";
    out << "\nKp: " << robot.Kp.transpose();
    out << "\nKi: " << robot.Ki.transpose();
    out << "\nKd: " << robot.Kd.transpose();
    out << "\n";
    return out;
}