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

void TestTfMatrixInSpaceFrame(ForwardKinematics* robot, VectorXd q, VectorXd r0) {
    robot->TfInSpaceFrame(q);
    MatrixXd tf_space = robot->GetTfSpace();
    std::cout << "\ntf_space:\n" << tf_space;

    std::cout << "\nr_in space frame: \n" << (tf_space * r0).transpose();
}

void TestTfMatrixInBodyFrame(ForwardKinematics* robot, VectorXd q, VectorXd r0) {
    robot->TfInBodyFrame(q);
    MatrixXd tf_body = robot->GetTfBody();
    std::cout << "\ntf_body:\n" << tf_body;

    std::cout << "\nr_in space frame: \n" << (tf_body * r0).transpose();
}

void TestSpaceJacobian(ForwardKinematics* robot, VectorXd q, VectorXd qd, VectorXd F) {
    MatrixXd space_jacobian = robot->SpaceJacobian(q);
    std::cout << "\nspace_jacobian:\n" << space_jacobian;

    VectorXd space_twist = robot->CalculateTwist(space_jacobian, qd);
    std::cout << "\nspace_twist:\n" << space_twist;

    VectorXd space_joint_torques = robot->CalculateJointTorques(space_jacobian, F);
    std::cout << "\nspace_joint_torques:\n" << space_joint_torques;
}

void TestBodyJacobian(ForwardKinematics* robot, VectorXd q, VectorXd qd, VectorXd F) {
    MatrixXd body_jacobian = robot->BodyJacobian(q);
    std::cout << "\nbody_jacobian:\n" << body_jacobian;

    VectorXd body_twist = robot->CalculateTwist(body_jacobian, qd);
    std::cout << "\nbody_twist:\n" << body_twist;

    VectorXd body_joint_torques = robot->CalculateJointTorques(body_jacobian, F);
    std::cout << "\nbody_joint_torques:\n" << body_joint_torques;

}

void TestRRrobotFK() {
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


    ForwardKinematics* rr_robot = new ForwardKinematics(num_joints,
                                                        joint_type, 
                                                        tf_home, 
                                                        screw_space, 
                                                        screw_body);

    std::cout << *rr_robot;

    VectorXd thetad(num_joints);
    thetad << 1.0, 1.0;

    VectorXd F(6);
    F(0) = 1;

    TestTfMatrixInSpaceFrame(rr_robot, theta, r0);

    // TestTfMatrixInBodyFrame(rr_robot, theta, r0);

    // TestSpaceJacobian(rr_robot, theta, thetad, F);

    // TestBodyJacobian(rr_robot, theta, thetad, F);

    delete rr_robot;
}


void TestRPRFK() {
    // set number of joints of robot
    int num_joints = 3; 
    // set joint types
    std::vector<std::string> joint_type = {"R", "P", "R"};
    // set home position: 
    // the position and orientation of end effector frame 
    // when all joint angles are set to zero
    MatrixXd tf_home(4, 4);
    tf_home << -1, 0,  0, 0,
                0, 1,  0, 6,
                0, 0, -1, 2,
                0, 0,  0, 1;

    MatrixXd screw_space(6,num_joints);
    screw_space << 0, 0, 0,
                   0, 0, 0,
                   1, 0, -1,
                   4, 0, -6,
                   0, 1, 0,
                   0, 0, -0.1;

    MatrixXd screw_body(6,num_joints);
    screw_body << 0, 0, 0,
                   0, 0, 0,
                  -1, 0, 1,
                   2, 0, 0,
                   0, 1, 0,
                   0, 0, 0.1;

    ForwardKinematics* rpr_robot = new ForwardKinematics(num_joints,
                                                        joint_type, 
                                                        tf_home, 
                                                        screw_space, 
                                                        screw_body);

    MatrixXd tf_expected(4,4);
    tf_expected << 0, 1,  0,         -5,
                  1, 0,  0,          4,
                  0, 0, -1, 1.68584073,
                  0, 0,  0,          1;

    std::cout << "\n tf_expected: \n" << tf_expected;

    VectorXd theta(num_joints);
    theta << M_PI/2, 3, M_PI;

    VectorXd r0(4);
    r0 << 0, 0, 0, 1;

    TestTfMatrixInSpaceFrame(rpr_robot, theta, r0);

    TestTfMatrixInBodyFrame(rpr_robot, theta, r0);

    delete rpr_robot;
}

int main() {
    // TestRRrobotFK();

    TestRPRFK();

    return 0;
}