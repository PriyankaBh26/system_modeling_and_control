# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "multi_joint_robots/serial_chain_robot_dynamics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void TestInverseDynamics(SerialChainRobotDynamics* fd,
                        int num_joints,
                        VectorXd q, 
                        VectorXd dq, 
                        VectorXd g,
                        VectorXd Ftip) {
    VectorXd d2q(num_joints);
    d2q << 2, 1.5, 1;

    VectorXd tau_expected(num_joints);
    tau_expected << 74.69616155, -33.06766016, -3.23057314;

    VectorXd tau_out = fd->InverseDynamics(q, dq, d2q, g, Ftip);

    std::cout << "\n TestInverseDynamics\n";
    std::cout << "\n tau_out - tau_expected :\n" << tau_out.transpose() - tau_expected.transpose();
    std::cout << "\n";

}

void TestMassMatrix(SerialChainRobotDynamics* fd, VectorXd q) {

        MatrixXd M_expected(3,3);
        M_expected << 2.25433380e+01, -3.07146754e-01, -7.18426391e-03,
                     -3.07146754e-01,  1.96850717e+00,  4.32157368e-01,
                     -7.18426391e-03,  4.32157368e-01,  1.91630858e-01;

        MatrixXd M = fd->MassMatrix(q);

        std::cout << "\n TestMassMatrix\n";
        std::cout << "\n M - M_expected :\n" << M - M_expected;
        std::cout << "\n";

}

void TestVelQuadraticForces(SerialChainRobotDynamics* fd, VectorXd q, VectorXd dq) {
        VectorXd C_expected(3);
        C_expected << 0.26453118, -0.05505157, -0.00689132;
        VectorXd C = fd->VelQuadraticForces(q, dq);

        std::cout << "\n TestVelQuadraticForces\n";
        std::cout << "\n C - C_expected :\n" << C.transpose() - C_expected.transpose();
        std::cout << "\n";

}

void TestGravityForces(SerialChainRobotDynamics* fd, VectorXd q, VectorXd dq) {
        VectorXd Gf_expected(3);
        Gf_expected << 28.40331262, -37.64094817, -5.4415892;
        VectorXd Gf = fd->GravityForces(q);

        std::cout << "\n TestGravityForces\n";
        std::cout << "\n Gf - Gf_expected :\n" << Gf.transpose() - Gf_expected.transpose();
        std::cout << "\n";

}

void TestEndEffectorForces(SerialChainRobotDynamics* fd, VectorXd q, VectorXd Ftip) {
        VectorXd Fee_expected(3);
        Fee_expected << 1.40954608, 1.85771497, 1.392409;
        VectorXd Fee = fd->EndEffectorForces(q, Ftip);

        std::cout << "\n TestEndEffectorForces\n";
        std::cout << "\n Fee - Fee_expected :\n" << Fee.transpose() - Fee_expected.transpose();
        std::cout << "\n";

}

SerialChainRobotDynamics* InitializeFD() {
    int num_joints = 3;

    VectorXd q(num_joints);
    q << 0.1, 0.1, 0.1;

    VectorXd dq(num_joints);
    dq << 0.1, 0.2, 0.3;

    VectorXd g(3);
    g << 0, 0, -9.8;

    VectorXd Ftip(6);
    Ftip << 1, 1, 1, 1, 1, 1;

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

    SerialChainRobotDynamics* fd = new SerialChainRobotDynamics(num_joints, g, Mlist, Glist, Slist);

        TestInverseDynamics(fd, num_joints, q, dq, g, Ftip);

        TestMassMatrix(fd, q);

        TestVelQuadraticForces(fd, q, dq);

        TestGravityForces(fd, q, dq);

        TestEndEffectorForces(fd, q, Ftip);
    return fd;
};

int main() {

    SerialChainRobotDynamics* fd = InitializeFD();

    delete fd;

    return 0;
}
