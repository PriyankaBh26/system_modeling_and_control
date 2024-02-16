# include <iostream>
# include <tuple>
# include <Eigen/Dense>

# include "serial_chain_robots/serial_chain_robot_pid_control.h"
// # include "data_logging/data_logging_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void TestSetPIDGains(SerialChainRobotPIDControl* robot) {
    VectorXd Kp_in(3);
    Kp_in << 1.3, 1.3, 1.3;

    VectorXd Ki_in(3);
    Ki_in << 1.2, 1.2, 1.2;

    VectorXd Kd_in(3);
    Kd_in << 1.1, 1.1, 1.1;

    robot->SetPIDGains(Kp_in, Ki_in, Kd_in);

    std::cout << "\n TestSetPIDGains \n";
    std::cout << *robot;
    std::cout << "\n";

}

void TestControlInput(SerialChainRobotPIDControl* robot, int num_joints) {
    VectorXd q_ref(num_joints);
    q_ref << 1.0, 1.0, 1.0;

    VectorXd dq_ref(num_joints);
    dq_ref << 2.0, 1.2, 2.0;

    VectorXd q(num_joints);
    q << 0.1, 0.1, 0.1;

    VectorXd dq(num_joints);
    dq << 0.1, 0.2, 0.3;

    VectorXd u = robot->ControlInput(q_ref, dq_ref, q, dq);

    std::cout << "\n TestControlInput \n";
    std::cout << "\n control input: " << u.transpose();
    std::cout << "\n";
}

void TestComputeTorque(SerialChainRobotPIDControl* robot, int num_joints) {
    VectorXd q_ref(num_joints);
    q_ref << 1.0, 1.0, 1.0;

    VectorXd dq_ref(num_joints);
    dq_ref << 2.0, 1.2, 2.0;

    VectorXd d2q_ref(num_joints);
    d2q_ref << 0.1, 0.1, 0.1;

    VectorXd q(num_joints);
    q << 0.1, 0.1, 0.1;

    VectorXd dq(num_joints);
    dq << 0.1, 0.2, 0.3;

    VectorXd tau = robot->ComputeTorque(q_ref, dq_ref, d2q_ref, q, dq);
    std::cout << "\n TestComputeTorque \n";
    std::cout << "\n tau: " << tau.transpose();
    std::cout << "\n";
}

void TestComputedTorqueControl(SerialChainRobotPIDControl* robot, int num_joints) {
    double dt = 1e-2;
    double dh = 1e-3;

    VectorXd Ftip = VectorXd::Ones(6);

    VectorXd q_ref(num_joints);
    q_ref << 1.0, 1.0, 1.0;

    VectorXd dq_ref(num_joints);
    dq_ref << 2.0, 1.2, 2.0;

    VectorXd d2q_ref(num_joints);
    d2q_ref << 0.1, 0.1, 0.1;

    VectorXd q(num_joints);
    q << 0.1, 0.1, 0.1;

    VectorXd dq(num_joints);
    dq << 0.1, 0.2, 0.3;

    VectorXd d2q(num_joints);
    VectorXd tau(num_joints);

    std::tie(q, dq, d2q, tau) = robot->ComputedTorqueControl(q_ref, dq_ref, d2q_ref, 
                                                             q, dq, Ftip, dt, dh);
}

void TestGetterFuns(SerialChainRobotPIDControl* robot) {
    
    std::cout << "\n TestGetterFuns \n";
    std::cout << "\n e history length: " << robot->GetQErrorHistory().size();
    std::cout << "\n sum_e history length: " << robot->GetSumErrorHistory().size();
    std::cout << "\n de history length: " << robot->GetDqErrorHistory().size();
    std::cout << "\n control input history length: " << robot->GetControlHistory().size();
    std::cout << "\n q history length: " << robot->GetQHistory().size();
    std::cout << "\n dq history length: " << robot->GetDqHistory().size();
    std::cout << "\n d2q history length: " << robot->GetD2qHistory().size();
    std::cout << "\n tau history length: " << robot->GetTauHistory().size();

    std::cout << "\n";

}


void TestSerialChainRobotPIDControl() {
    int num_joints = 3;

    VectorXd g(3);
    g << 0, 0, -9.8;

    MatrixXd M01(4,4);
    M01 << 1, 0, 0,        0,
            0, 1, 0,        0,
            0, 0, 1, 0.089159,
            0, 0, 0,        1;

    MatrixXd M12(4,4);
    M12 <<  0, 0, 1,    0.28,
             0, 1, 0, 0.13585,
            -1, 0, 0,       0,
             0, 0, 0,       1;

    MatrixXd M23(4,4);
    M23 << 1, 0, 0,       0,
            0, 1, 0, -0.1197,
            0, 0, 1,   0.395,
            0, 0, 0,       1;

    MatrixXd M34(4,4);
    M34 << 1, 0, 0,       0,
            0, 1, 0,       0,
            0, 0, 1, 0.14225,
            0, 0, 0,       1;

    VectorXd G1_diag(6);
    G1_diag << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
    MatrixXd G1 = G1_diag.asDiagonal();

    VectorXd G2_diag(6);
    G2_diag << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
    MatrixXd G2 = G2_diag.asDiagonal();

    VectorXd G3_diag(6);
    G3_diag << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;
    MatrixXd G3 = G3_diag.asDiagonal();

    std::vector<MatrixXd> Glist = {G1, G2, G3};

    std::vector<MatrixXd> Mlist = {M01, M12, M23, M34};

    MatrixXd Slist(6,num_joints);
    Slist <<    1, 0, 0,
                0, 1, 1,
                1, 0, 0,
                0, -0.089, -0.089,
                1, 0, 0,
                0, 0, 0.425;

    SerialChainRobotPIDControl* robot = new SerialChainRobotPIDControl(num_joints, g, Mlist, Glist, Slist);

    TestSetPIDGains(robot);

    TestControlInput(robot, num_joints);

    TestComputeTorque(robot, num_joints);

    TestComputedTorqueControl(robot, num_joints);

    TestGetterFuns(robot);

    delete robot;

};


int main() {

    TestSerialChainRobotPIDControl();

    return 0;
}