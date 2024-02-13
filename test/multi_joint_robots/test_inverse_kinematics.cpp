# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "multi_joint_robots/inverse_kinematics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;

VectorXd GetXdesired(VectorXd L, VectorXd theta, int num_joints) {
    // end effector position and rotation angle wrt x-axis
    double x = 0;
    double y = 0;
    double z = 0;
    double phi = 0;
    for (int i{0}; i<num_joints; i++) {
        phi += theta(i);
        x += L(i) * cos(phi);
        y += L(i) * sin(phi);
    }
    std::cout << "\n Num of joints of robot: " << num_joints << "\n";
    std::cout << "x: " << x << "\n";
    std::cout << "y: " << y << "\n";
    std::cout << "z: " << z << "\n";
    std::cout << "phi: " << phi << "\n";
    VectorXd xd(6);
    xd << x, y, z, phi, M_PI/2 - phi, 0;
    return xd;
}

MatrixXd ProductOfTFMatrices(VectorXd L, VectorXd theta, VectorXd r0) {
    // create TF mat:
    Matrix3d R1;
    R1 = AngleAxisd(theta(0), Vector3d::UnitZ());
    Matrix3d R2;
    R2 = AngleAxisd(theta(1), Vector3d::UnitZ());

    // T03 = T01 * T12 * T23
    MatrixXd T03(4,4);
    MatrixXd T01 = MatrixXd::Identity(4,4);
    T01.block(0,0,3,3) = R1;
    MatrixXd T12 = MatrixXd::Identity(4,4);
    T12.block(0,0,3,3) = R2;
    T12(0,3) = L(0);
    MatrixXd T23 = MatrixXd::Identity(4,4);
    T23(0,3) = L(1);

    T03 = T01 * T12 * T23;
    // std::cout << "\n T03 expected:\n" << T03;

    std::cout << "\nr expected: \n" << (T03 * r0).transpose();
    return T03;
}

void TestTfSpaceToBody(InverseKinematics* inv_kin, MatrixXd tf_expected, VectorXd q0) {
    MatrixXd tf_body = inv_kin->TfSpaceToBody(tf_expected, q0);
    std::cout << "\nTestTfSpaceToBody\n";
    std::cout << "\n tf_body: \n" << tf_body;
    std::cout << "\n";
}

void TestSkewSymMatToVec(InverseKinematics* inv_kin) {
    MatrixXd W(3,3);
    W << 0, -3,  2,
         3,  0, -1,
        -2,  1,  0;
    VectorXd w = inv_kin->SkewSymMatToVec(W);
    VectorXd w_expected(3);
    w_expected << 1, 2, 3;
    std::cout << "\nTestSkewSymMatToVec\n";
    std::cout << "\n w - w_expected : " << w.transpose() - w_expected.transpose();
    std::cout << "\n";

}

void TestVecToSkewSymMat(InverseKinematics* inv_kin) {
    MatrixXd W_expected(3,3);
    W_expected << 0, -3,  2,
                  3,  0, -1,
                 -2,  1,  0;
    VectorXd w(3);
    w << 1, 2, 3;
    MatrixXd W = inv_kin->VecToSkewSymMat(w);
    std::cout << "\nTestVecToSkewSymMat\n";
    std::cout << "\n W - W_expected : " << W - W_expected;
    std::cout << "\n";

}

void TestMatrixLog3(InverseKinematics* inv_kin) {
    MatrixXd R(3,3);
    R << 0, 0, 1,
         1, 0, 0,
         0, 1, 0;
    
    MatrixXd W_expected(3,3);
    W_expected << 0, -1.20919958,  1.20919958,
                  1.20919958,    0, -1.20919958,
                 -1.20919958,  1.20919958,  0;

    MatrixXd W = inv_kin->MatrixLog3(R);
    std::cout << "\nTestMatrixLog3\n";
    std::cout << "\n W - W_expected\n: " << W - W_expected;
    std::cout << "\n";
}

void TestSe3ToVec(InverseKinematics* inv_kin) {
    MatrixXd V_B(4,4);
    V_B <<   0, -3,  2, 4,
             3,  0, -1, 5,
            -2,  1,  0, 6,
             0,  0,  0, 0;

    VectorXd V_b_expected(6);
    V_b_expected << 1, 2, 3, 4, 5, 6;
    VectorXd V_b = inv_kin->Se3ToVec(V_B);
    std::cout << "\nTestSe3ToVec\n";
    std::cout << "\n V_b - V_b_expected : " << V_b.transpose() - V_b_expected.transpose();
    std::cout << "\n";
}

int main() {

    int num_joints = 2; 
    double tolerance = 1e-4;
    int max_iterations = 100;

    // set link lengths
    VectorXd L(num_joints);
    L << 1.0, 1.5;
    // set joint types
    std::vector<std::string> joint_type = {"R", "R"};
    // set link angle of rotation
    VectorXd theta(num_joints);
    theta << M_PI/180 * 30, M_PI/180 * 30;

    VectorXd q0 = theta.array();

    VectorXd x_d = GetXdesired(L, theta, num_joints);

    VectorXd r0(4);
    r0 << 0, 0, 0, 1;
    
    MatrixXd tf_expected = ProductOfTFMatrices(L, theta, r0);
    std::cout << "\n tf_desired : \n" << tf_expected;
    // set home position: 
    // the position and orientation of end effector frame 
    // when all joint angles are set to zero
    MatrixXd tf_home = MatrixXd::Identity(4, 4);
    tf_home(0,3) = L.sum();

    MatrixXd screw_space(6,num_joints+1);
    screw_space << 0, 0, 0,
                    0, 0, 0,
                    1, 1, 1,
                    0, 0, 0,
                    0, -L(0), -L.sum(),
                    0, 0, 0;

    MatrixXd screw_body(6,num_joints+1);
    screw_body << 0, 0, 0,
                    0, 0, 0,
                    1, 1, 1,
                    0, 0, 0,
                    L.sum(), L(1), 0,
                    0, 0, 0;

    InverseKinematics* inv_kin = new InverseKinematics(num_joints, 
                                                        joint_type, 
                                                        tf_home, 
                                                        screw_space, 
                                                        screw_body,
                                                        "body",
                                                        tf_expected,
                                                        q0,
                                                        tolerance,
                                                        max_iterations);
    
    std::cout << *inv_kin;

    TestTfSpaceToBody(inv_kin, tf_expected, q0);

    TestSkewSymMatToVec(inv_kin);

    TestVecToSkewSymMat(inv_kin);

    TestMatrixLog3(inv_kin);

    TestSe3ToVec(inv_kin);

    // VectorXd qd = inv_kin->SolveIK();

    // std::cout << "\n q_desired: \n" << qd.transpose();

    delete inv_kin;

    return 0;
}