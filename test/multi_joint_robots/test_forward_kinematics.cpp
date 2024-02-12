# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "multi_joint_robots/forward_kinematics.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    // set number of joints of robot
    int num_joints = 2; 
    // set link lengths
    VectorXd link_length(num_joints);
    link_length << 1.0, 1.5;
    // set joint types
    std::vector<std::string> joint_type = {"R", "R"};
    // set link angle of rotation
    VectorXd theta(num_joints);
    theta << 30, 60;
    // end effector position and rotation angle wrt x-axis
    double x = 0;
    double y = 0;
    double z = 0;
    double phi = 0;
    for (int i{0}; i<num_joints; i++) {
        phi += theta(i);
        x += link_length(i) * cos(M_PI/180 * phi);
        y += link_length(i) * sin(M_PI/180 * phi);
    }
    std::cout << "\n Num of joints of robot: " << num_joints << "\n";
    std::cout << "x: " << x << "\n";
    std::cout << "y: " << y << "\n";
    std::cout << "z: " << z << "\n";
    std::cout << "phi: " << phi << "\n";

    // set home position: 
    // the position and orientation of end effector frame 
    // when all joint angles are set to zero
    MatrixXd tf_0_se = MatrixXd::Identity(4, 4);
    tf_0_se(0,3) = link_length.sum();
    std::cout << "\nHome position TF Mat:\n " << tf_0_se << "\n";

    MatrixXd screw_axes_s; 
    MatrixXd screw_axes_e;

    // ForwardKinematics* two_joint_robot = new ForwardKinematics(num_joints, link_length, 
    //                                                           joint_type, tf_0_se, 
    //                                                           screw_axes_s, screw_axes_e);


    // delete two_joint_robot;

    return 0;
}