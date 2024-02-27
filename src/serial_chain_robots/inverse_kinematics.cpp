# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "serial_chain_robots/serial_chain_robot_helper_funs.h"
# include "serial_chain_robots/inverse_kinematics.h"
# include "serial_chain_robots/forward_kinematics.h"
# include "numerical_solvers/newton_raphson.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// desired end-effector configuration represented as tf_desired is given

InverseKinematics::InverseKinematics(
    int num_joints, 
    std::vector<std::string> joint_type, 
    MatrixXd tf_home, 
    MatrixXd screw_space, 
    MatrixXd screw_body,
    std::string str,
    double tolerance,
    int max_iterations) : ForwardKinematics(num_joints, 
                                            joint_type,
                                            tf_home,
                                            screw_space,
                                            screw_body), 

                          NewtonRaphson(num_joints,
                                        6,
                                        tolerance,
                                        max_iterations),

                                        desired_config_type(str),
                                        tf_desired(MatrixXd::Identity(4,4)),
                                        q(VectorXd::Zero(num_joints)) {};

MatrixXd InverseKinematics::TfBody(VectorXd q) {
    TfInBodyFrame(q);
    MatrixXd tf_mat_ = GetTfBody();
    MatrixXd tf_mat = TfmatInverse(tf_mat_) * tf_desired;
    return tf_mat;
};

MatrixXd InverseKinematics::TfSpace(VectorXd q) {
    TfInSpaceFrame(q);
    MatrixXd tf_mat_ = GetTfSpace();
    MatrixXd tf_mat = TfmatInverse(tf_mat_) * tf_desired;
    return tf_mat;
};

VectorXd InverseKinematics::f(VectorXd q) {
    VectorXd V(4);
    if (desired_config_type == "body_frame") {
        MatrixXd tf_body_ = InverseKinematics::TfBody(q);
        MatrixXd V_B = MatrixLog6(tf_body_);
        V = Se3ToVec(V_B);
    } else if (desired_config_type == "space_frame") {
        MatrixXd tf_body_ = InverseKinematics::TfSpace(q);
        MatrixXd V_B = MatrixLog6(tf_body_);
        VectorXd V_b = Se3ToVec(V_B);
        V = AdjointOfTfMatrix(GetTfSpace()) * V_b;
    }
    return V;
};

MatrixXd InverseKinematics::dfdq(VectorXd q) {
    MatrixXd mat(6,q.cols());
    if (desired_config_type == "body_frame") {
        mat = -BodyJacobian(q);
    } else if (desired_config_type == "space_frame") {
        mat = -SpaceJacobian(q);
    }
    return mat;
};

void InverseKinematics::SetTfDesired(MatrixXd tf_des) {
    tf_desired = tf_des;
}

void InverseKinematics::SetInitialJointAngles(VectorXd q0) {
    q = q0;
}

VectorXd InverseKinematics::SolveIK(MatrixXd tf_des, 
                                    VectorXd q0) {
    SetTfDesired(tf_des);
    SetInitialJointAngles(q0);

    Iterate(q0);

    std::vector<VectorXd> q_history = GetSolutionHistory();

    return q_history.back();
};

std::ostream& operator << (std::ostream& out, InverseKinematics& system) {
    out << "\n Inverse Kinematics parameters :\n";
    return out;
};
