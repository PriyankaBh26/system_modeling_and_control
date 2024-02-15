# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "serial_chain_robots/inverse_kinematics.h"

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

MatrixXd ProductOfTFMatrices(VectorXd L, VectorXd theta) {
    VectorXd r0(4);
    r0 << 0, 0, 0, 1;
    
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

    // std::cout << "\nr expected: \n" << (T03 * r0).transpose();
    return T03;
}

void TestTfBody(InverseKinematics* inv_kin, MatrixXd tf_expected, VectorXd q0) {
    MatrixXd tf_body = inv_kin->TfBody(q0);
    std::cout << "\nTestBody\n";
    std::cout << "\n tf_body: \n" << tf_body;
    std::cout << "\n";
}

void TestTfSpace(InverseKinematics* inv_kin, MatrixXd tf_expected, VectorXd q0) {
    MatrixXd tf_space = inv_kin->TfSpace(q0);
    std::cout << "\nTestspace\n";
    std::cout << "\n tf_space: \n" << tf_space;
    std::cout << "\n";
}

void Testf(InverseKinematics* inv_kin, VectorXd q) {
    VectorXd V_b = inv_kin->f(q);
    
    std::cout << "\nTestf\n";
    std::cout << "\n V_b\n " << V_b;
    std::cout << "\n";
}

void Testdfdq(InverseKinematics* inv_kin, VectorXd q) {
    MatrixXd Jac = inv_kin->dfdq(q);
    std::cout << "\nTestdfdq\n";
    std::cout << "\n Jacobian:\n " << Jac;
    std::cout << "\n";
}

void TestSolveIK(InverseKinematics* inv_kin,
                 VectorXd L,
                 VectorXd theta) {

    MatrixXd tf_desired = ProductOfTFMatrices(L, theta);

    VectorXd q0 = VectorXd::Random(2);

    std::cout << "\n TestSolveIK \n";
    std::cout << "\n";

    VectorXd q_result = inv_kin->SolveIK(tf_desired, q0);
    std::cout << "\nq_expected:" << theta.transpose();
    std::cout << "\nq_result:" << q_result.transpose();
}

int main() {

    int num_joints = 2; 
    double tolerance = 1e-4;
    int max_iterations = 100;

    std::string desired_config_type = "body_frame";

    // set link lengths
    VectorXd L(num_joints);
    L << 1.0, 1.5;
    // set joint types
    std::vector<std::string> joint_type = {"R", "R"};

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
                                                        desired_config_type,
                                                        tolerance,
                                                        max_iterations);
    
    std::cout << *inv_kin;

    // set link angle of rotation
    VectorXd theta(num_joints);
    theta << M_PI/180 * 30, M_PI/180 * 30;

    TestSolveIK(inv_kin, L, theta);

    // set link angle of rotation
    VectorXd theta1(num_joints);
    theta1 << M_PI/180 * 50, M_PI/180 * 40;

    TestSolveIK(inv_kin, L, theta1);

    // set link angle of rotation
    VectorXd theta2(num_joints);
    theta2 << M_PI/180 * 20, M_PI/180 * 90;

    TestSolveIK(inv_kin, L, theta2);

    delete inv_kin;

    return 0;
}