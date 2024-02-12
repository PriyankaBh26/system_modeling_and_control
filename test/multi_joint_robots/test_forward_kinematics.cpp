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

void TestTfMatrixInSpaceFrame(ForwardKinematics* robot, VectorXd q, VectorXd r0, MatrixXd tf_space_expected) {
    robot->TfInSpaceFrame(q);
    MatrixXd tf_space = robot->GetTfSpace();
    std::cout << "\ntf_space - tf_space_expected:\n" << tf_space - tf_space_expected;

    std::cout << "\ntf * r: \n" << (tf_space * r0).transpose();
}

void TestTfMatrixInBodyFrame(ForwardKinematics* robot, VectorXd q, VectorXd r0, MatrixXd tf_body_expected) {
    robot->TfInBodyFrame(q);
    MatrixXd tf_body = robot->GetTfBody();
    std::cout << "\ntf_body - tf_body_expected:\n" << tf_body - tf_body_expected;

    std::cout << "\ntf * r: \n" << (tf_body * r0).transpose();
}

void TestSpaceJacobian(ForwardKinematics* robot, VectorXd q, VectorXd qd, VectorXd F, MatrixXd space_jacobian_expected) {
    MatrixXd space_jacobian = robot->SpaceJacobian(q);
    std::cout << "\nspace_jacobian - space_jacobian_expected:\n" << space_jacobian - space_jacobian_expected;

    VectorXd space_twist = robot->CalculateTwist(space_jacobian, qd);
    std::cout << "\nspace_twist:\n" << space_twist.transpose();

    VectorXd space_joint_torques = robot->CalculateJointTorques(space_jacobian, F);
    std::cout << "\nspace_joint_torques:\n" << space_joint_torques.transpose();
}

void TestBodyJacobian(ForwardKinematics* robot, VectorXd q, VectorXd qd, VectorXd F, MatrixXd body_jacobian_expected) {
    MatrixXd body_jacobian = robot->BodyJacobian(q);
    std::cout << "\nbody_jacobian - body_jacobian_expected:\n" << body_jacobian - body_jacobian_expected;

    VectorXd body_twist = robot->CalculateTwist(body_jacobian, qd);
    std::cout << "\nbody_twist:\n" << body_twist.transpose();

    VectorXd body_joint_torques = robot->CalculateJointTorques(body_jacobian, F);
    std::cout << "\nbody_joint_torques:\n" << body_joint_torques.transpose();

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
    
    MatrixXd tf_expecetd = ProductOfTFMatrices(L, theta, r0);

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

    TestTfMatrixInSpaceFrame(rr_robot, theta, r0, tf_expecetd);

    TestTfMatrixInBodyFrame(rr_robot, theta, r0, tf_expecetd);

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
    std::cout << *rpr_robot;

    MatrixXd tf_expected(4,4);
    tf_expected << 0, 1,  0,         -5,
                  1, 0,  0,          4,
                  0, 0, -1, 1.68584073,
                  0, 0,  0,          1;

    // std::cout << "\n tf_expected: \n" << tf_expected;

    VectorXd theta(num_joints);
    theta << M_PI/2, 3, M_PI;

    VectorXd r0(4);
    r0 << 0, 0, 0, 1;

    TestTfMatrixInSpaceFrame(rpr_robot, theta, r0, tf_expected);

    TestTfMatrixInBodyFrame(rpr_robot, theta, r0, tf_expected);

    delete rpr_robot;
}

void TestRRRRJacobian() {
    // set number of joints of robot
    int num_joints = 4; 
    // set joint types
    std::vector<std::string> joint_type = {"R", "R", "R", "R"};
    // set home position: 
    // the position and orientation of end effector frame 
    // when all joint angles are set to zero
    MatrixXd tf_home(4, 4);
    tf_home << -1, 0,  0, 0,
                0, 1,  0, 6,
                0, 0, -1, 2,
                0, 0,  0, 1;

    MatrixXd screw_space(6,num_joints);
    screw_space << 0, 1, 0, 1,
                   0, 0, 1, 0,
                   1, 0, 0, 0,
                   0, 2, 0, 0.2,
                 0.2, 0, 2, 0.3,
                 0.2, 3, 1, 0.4;

    MatrixXd screw_body(6,num_joints);
    screw_body << 0, 1, 0, 1,
                   0, 0, 1, 0,
                   1, 0, 0, 0,
                   0, 2, 0, 0.2,
                 0.2, 0, 2, 0.3,
                 0.2, 3, 1, 0.4;

    MatrixXd body_jac_expected(6, num_joints);
    body_jac_expected << -0.04528405, 0.99500417,           0,   1,
                          0.74359313, 0.09304865,  0.36235775,   0,
                         -0.66709716, 0.03617541, -0.93203909,   0,
                          2.32586047,    1.66809,  0.56410831, 0.2,
                         -1.44321167, 2.94561275,  1.43306521, 0.3,
                         -2.06639565, 1.82881722, -1.58868628, 0.4;

    MatrixXd space_jac_expected(6, num_joints);
    space_jac_expected <<   0, 0.98006658, -0.09011564,  0.95749426,
                            0, 0.19866933,   0.4445544,  0.28487557,
                            1,          0,  0.89120736, -0.04528405,
                            0, 1.95218638, -2.21635216, -0.51161537,
                          0.2, 0.43654132, -2.43712573,  2.77535713,
                          0.2, 2.96026613,  3.23573065,  2.22512443;

    ForwardKinematics* rrrr_robot = new ForwardKinematics(num_joints,
                                                        joint_type, 
                                                        tf_home, 
                                                        screw_space, 
                                                        screw_body);
    std::cout << *rrrr_robot;

    VectorXd theta(num_joints);
    theta << 0.2, 1.1, 0.1, 1.2;

    VectorXd thetad(num_joints);
    thetad << 0.2, 1.1, 0.1, 1.2;

    VectorXd F(6);
    F(0) = 1;

    TestSpaceJacobian(rrrr_robot, theta, thetad, F, space_jac_expected);

    TestBodyJacobian(rrrr_robot, theta, thetad, F, body_jac_expected);

    delete rrrr_robot;
}

int main() {
    TestRRrobotFK();

    TestRPRFK();

    TestRRRRJacobian();

    return 0;
}