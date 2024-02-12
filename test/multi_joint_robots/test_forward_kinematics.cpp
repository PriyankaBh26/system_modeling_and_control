# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "multi_joint_robots/forward_kinematics.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::AngleAxisd;

void InitializeXYZPhi(VectorXd L, VectorXd theta, int num_joints) {
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
}

void ProductOfTFMatrices(VectorXd L, VectorXd theta, VectorXd r0) {
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
    std::cout << "\n T03 expected:\n" << T03;

    std::cout << "\nr expected: \n" << (T03 * r0).transpose();
}

void TestTfMatrixInBaseFrame(ForwardKinematics* two_joint_robot, VectorXd q, VectorXd r0) {
    two_joint_robot->TfMatrixInBaseFrame(q);
    MatrixXd tf_se = two_joint_robot->GetTfse();
    std::cout << "\ntf_se:\n" << tf_se;

    std::cout << "\nr_in space frame: \n" << (tf_se * r0).transpose();
}

void TestTfMatrixInEEFrame(ForwardKinematics* two_joint_robot, VectorXd q, VectorXd r0) {
    two_joint_robot->TfMatrixInEEFrame(q);
    MatrixXd tf_es = two_joint_robot->GetTfes();
    std::cout << "\ntf_es:\n" << tf_es;

    std::cout << "\nr_in space frame: \n" << (tf_es * r0).transpose();
}

int main() {
        // set number of joints of robot
    int num_joints = 2; 
    // set link lengths
    VectorXd L(num_joints);
    L << 1.0, 1.5;
    // set joint types
    std::vector<std::string> joint_type = {"R", "R"};
    // set link angle of rotation
    VectorXd theta(num_joints);
    theta << M_PI/180 * 30, M_PI/180 * 30;

    InitializeXYZPhi(L, theta, num_joints);

    VectorXd r0(4);
    r0 << 0, 0, 0, 1;
    
    ProductOfTFMatrices(L, theta, r0);

    // set home position: 
    // the position and orientation of end effector frame 
    // when all joint angles are set to zero
    MatrixXd tf_0_se = MatrixXd::Identity(4, 4);
    tf_0_se(0,3) = L.sum();

    MatrixXd screw_axes_s(6,num_joints+1);
    screw_axes_s << 0, 0, 0,
                    0, 0, 0,
                    1, 1, 1,
                    0, 0, 0,
                    0, -L(0), -L.sum(),
                    0, 0, 0;

    MatrixXd screw_axes_e(6,num_joints+1);
    screw_axes_e << 0, 0, 0,
                    0, 0, 0,
                    1, 1, 1,
                    0, 0, 0,
                    L.sum(), L(1), 0,
                    0, 0, 0;

    ForwardKinematics* two_joint_robot = new ForwardKinematics(num_joints, L, 
                                                              joint_type, tf_0_se, 
                                                              screw_axes_s, screw_axes_e);

    TestTfMatrixInBaseFrame(two_joint_robot, theta, r0);

    TestTfMatrixInEEFrame(two_joint_robot, theta, r0);

    delete two_joint_robot;


    return 0;
}