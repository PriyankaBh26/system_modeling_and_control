# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "multi_joint_robots/forward_kinematics.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;

ForwardKinematics::ForwardKinematics(int n,
                    std::vector<std::string> joint_type, 
                    MatrixXd m1, MatrixXd m2, MatrixXd m3) : num_joints(n), 
                                                            joint_type(joint_type), 
                                                            tf_home(m1), 
                                                            screw_space(m2), 
                                                            screw_body(m3), 
                                                            tf_space(MatrixXd::Identity(4,4)), 
                                                            tf_body(MatrixXd::Identity(4,4)) {};

void ForwardKinematics::TfInSpaceFrame(VectorXd q) {
    // tf_space = exp([s1]q1)*exp([s2]q2)*...*exp([sn]qn)*tf_home
    tf_space = MatrixXd::Identity(4,4);
    for (int i(0); i<num_joints; i++) {
        tf_space = tf_space * ForwardKinematics::ExponentialMatrix(screw_space.col(i), 
                                                    q(i), joint_type[i]);
    }
    tf_space = tf_space * tf_home;
};

void ForwardKinematics::TfInBodyFrame(VectorXd q) {
    // tf_body_k = tf_home*exp([B1]q1)*exp([B2]q2)*...*exp([Bn]qn)
    tf_body = tf_home;
    for (int i(0); i<num_joints; i++) {
        tf_body = tf_body * ForwardKinematics::ExponentialMatrix(screw_body.col(i), 
                                                            q(i), joint_type[i]);
    }
};     

MatrixXd ForwardKinematics::KthJointMatInSpaceFrame(VectorXd q, int k) {
    // tf_space_k = exp([s1]q1)*exp([s2]q2)*...*exp([sk]qk)
    MatrixXd tf_space_k = MatrixXd::Identity(4,4);
    for (int i(0); i<k; i++) {
        tf_space_k = tf_space_k * ForwardKinematics::ExponentialMatrix(screw_space.col(i), 
                                                                    q(i), joint_type[i]);
    }
    return tf_space_k;
};

MatrixXd ForwardKinematics::KthJointMatInBodyFrame(VectorXd q, int k) {
    // tf_body_k = exp(-[Bn]qn)*...*exp(-[Bk]qk)
    MatrixXd tf_body_k = MatrixXd::Identity(4,4);
    for (int i(num_joints-1); i>=k; i--) {
        tf_body_k = tf_body_k * ForwardKinematics::ExponentialMatrix(-screw_body.col(i), 
                                                                    q(i), joint_type[i]);
    }
    return tf_body_k;
};

MatrixXd ForwardKinematics::ExponentialMatrix(VectorXd screw_axis, 
                                              double q_i, 
                                              std::string joint_type_i) {
    // angular velocity axis
    VectorXd w(3);
    w << screw_axis(0), screw_axis(1), screw_axis(2);
    // linear velocity
    VectorXd v(3);
    v << screw_axis(3), screw_axis(4), screw_axis(5);

    MatrixXd exp_q(4,4);

    if (joint_type_i == "R") {
        // revolute joint
        MatrixXd W = ForwardKinematics::VecToSkewSymmetricMat(w);

        exp_q.block(0,0,3,3) = MatrixXd::Identity(3,3) + sin(q_i) * W + (1- cos(q_i)) * W * W;

        exp_q.block(0,3,3,1) = (MatrixXd::Identity(3,3) * q_i + (1 - cos(q_i)) * W + (q_i - sin(q_i)) * W * W) * v;
    
    } else if (joint_type_i == "P") {
        // prismatic joint
        exp_q.block(0,0,3,3) = MatrixXd::Identity(3,3);

        exp_q.block(0,3,3,1) = v * q_i;
    }
    exp_q(3,3) = 1;
    return exp_q;
};

MatrixXd ForwardKinematics::RotMatPosToTFMat(MatrixXd R, VectorXd p) {
    MatrixXd TFMat(4,4);
    TFMat.block(0,0,3,3) = R;
    TFMat.block(0,3,3,1) = p;
    TFMat(3,3) = 1;
    return TFMat;
}

MatrixXd ForwardKinematics::SpaceJacobian(VectorXd q) {
    MatrixXd space_jacobian(6,num_joints);
    space_jacobian.col(0) = screw_space.col(0);
    for (int k{1}; k<num_joints; k++) {
        MatrixXd tf_space_k = ForwardKinematics::KthJointMatInSpaceFrame(q, k);
        space_jacobian.col(k) = ForwardKinematics::AdjointOfTfMatrix(tf_space_k) * screw_space.col(k);
    }
    return space_jacobian;
};

MatrixXd ForwardKinematics::BodyJacobian(VectorXd q) {
    MatrixXd body_jacobian(6,num_joints);
    body_jacobian.col(num_joints-1) = screw_body.col(num_joints-1);
    for (int i{num_joints-2}; i>=0; i--) {
        MatrixXd tf_body_k = ForwardKinematics::KthJointMatInBodyFrame(q, i+1);
        body_jacobian.col(i) = ForwardKinematics::AdjointOfTfMatrix(tf_body_k) * screw_body.col(i);
    }
    return body_jacobian;
};

MatrixXd ForwardKinematics::AdjointOfTfMatrix(MatrixXd tf_mat) {
    MatrixXd ad_tf_mat(6,6);
    MatrixXd R(3,3);
    R = tf_mat.block(0,0,3,3);

    VectorXd p(3);
    p << tf_mat(0,3), tf_mat(1,3), tf_mat(2,3);

    MatrixXd p_mat = ForwardKinematics::VecToSkewSymmetricMat(p);

    ad_tf_mat.block(0,0,3,3) = R;
    ad_tf_mat.block(3,3,3,3) = R;
    ad_tf_mat.block(3,0,3,3) = p_mat * R;
    return ad_tf_mat;
};

VectorXd ForwardKinematics::CalculateTwist(MatrixXd jacobian, VectorXd qd) {
    VectorXd twist = jacobian * qd;
    return twist;
};

VectorXd ForwardKinematics::CalculateJointTorques(MatrixXd jacobian, VectorXd F) {
    VectorXd joint_torque = jacobian.transpose() * F;
    return joint_torque;
};

MatrixXd ForwardKinematics::VecToSkewSymmetricMat(VectorXd v) {
    MatrixXd skew_sym_matrix(3,3);
    skew_sym_matrix << 0, -v(2), v(1),
                        v(2), 0, -v(0),
                        -v(1), v(0), 0;

    return skew_sym_matrix;
};

MatrixXd ForwardKinematics::GetTfSpace() {return tf_space;};

MatrixXd ForwardKinematics::GetTfBody() {return tf_body;};

std::ostream& operator << (std::ostream&  out, ForwardKinematics& robot) {
    out << "\n Robot parameters:";
    out << "\nnumber of joints: " << robot.num_joints;
    out << "\njoint types: ";
    for (const auto& j : robot.joint_type) {
        out << j << " ";
    }
    out << "\n TF_home:\n";
    out << robot.tf_home << "\n";
    return out;
}