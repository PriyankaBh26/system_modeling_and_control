# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "multi_joint_robots/forward_dynamics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void TestInverseDynamics(ForwardDynamics* fd,
                        int num_joints,
                        VectorXd q, 
                        VectorXd dq, 
                        VectorXd g, 
                        VectorXd Ftip, 
                        std::vector<MatrixXd> Mlist, 
                        std::vector<MatrixXd> Glist, 
                        MatrixXd screw_space) {
    VectorXd d2q(num_joints);
    dq << 2, 1.5, 1;

    VectorXd tau_expected(num_joints);
    tau_expected << 74.69616155, -33.06766016, -3.23057314;

    VectorXd tau_out = fd->InverseDynamics(q, dq, d2q, 
                                        g, Ftip, Mlist, 
                                        Glist, screw_space);

    std::cout << "\n TestInverseDynamics\n";
    std::cout << "\n tau_out - tau_expected :\n" << tau_out.transpose();  // - tau_expected.transpose();
    std::cout << "\n";

}

void TestMassMatrix(ForwardDynamics* fd,
                VectorXd q,
                std::vector<MatrixXd> Mlist, 
                std::vector<MatrixXd> Glist, 
                MatrixXd screw_space) {

        MatrixXd M_expected(3,3);
        M_expected << 2.25433380e+01, -3.07146754e-01, -7.18426391e-03,
                     -3.07146754e-01,  1.96850717e+00,  4.32157368e-01,
                     -7.18426391e-03,  4.32157368e-01,  1.91630858e-01;

        MatrixXd M = fd->MassMatrix(q,
                                Mlist, 
                                Glist, 
                                screw_space);

        std::cout << "\n TestMassMatrix\n";
        std::cout << "\n M - M_expected :\n" << M - M_expected;
        std::cout << "\n";

}

ForwardDynamics* InitializeFD() {
    int num_joints = 3;

    VectorXd q(num_joints);
    q << 0.1, 0.1, 0.1;

    VectorXd dq(num_joints);
    dq << 0.1, 0.2, 0.3;

    VectorXd tau(num_joints);
    tau << 0.5, 0.6, 0.7;

    VectorXd g(3);
    g << 0, 0, -9.81;

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

    ForwardDynamics* fd = new ForwardDynamics(num_joints,
                                              q,
                                              dq,
                                              tau,
                                              g,
                                              Ftip,
                                              Mlist,
                                              Glist,
                                              Slist);

        TestInverseDynamics(fd,
                                num_joints,
                                q,
                                dq,
                                g,
                                Ftip,
                                Mlist,
                                Glist,
                                Slist);

        TestMassMatrix(fd,
                        q,
                        Mlist, 
                        Glist, 
                        Slist);


    return fd;
};

int main() {

    ForwardDynamics* fd = InitializeFD();

    delete fd;

    return 0;
}
