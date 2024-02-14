# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "multi_joint_robots/inverse_kinematics.h"
# include "multi_joint_robots/forward_kinematics.h"
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



MatrixXd InverseKinematics::TfmatInverse(MatrixXd Mat) {
    MatrixXd Mat_inverse(4,4);
    MatrixXd R = Mat.block(0,0,3,3);
    VectorXd p = Mat.block(0,3,3,1);

    Mat_inverse.block(0,0,3,3) = R.transpose();
    Mat_inverse.block(0,3,3,1) = -R.transpose() * p;
    Mat_inverse(3,3) = 1;
    return Mat_inverse;
}

MatrixXd InverseKinematics::TfBody(VectorXd q) {
    TfInBodyFrame(q);
    MatrixXd tf_mat_ = GetTfBody();
    MatrixXd tf_mat = InverseKinematics::TfmatInverse(tf_mat_) * tf_desired;
    return tf_mat;
};

MatrixXd InverseKinematics::TfSpace(VectorXd q) {
    TfInSpaceFrame(q);
    MatrixXd tf_mat_ = GetTfSpace();
    MatrixXd tf_mat = InverseKinematics::TfmatInverse(tf_mat_) * tf_desired;
    return tf_mat;
};

VectorXd InverseKinematics::SkewSymMatToVec(MatrixXd W) {
    VectorXd w_theta(3);
    w_theta << W(2,1), W(0,2), W(1,0);
    return w_theta;
};

MatrixXd InverseKinematics::VecToSkewSymMat(VectorXd w) {
    MatrixXd W(3,3);
    W << 0, -w(2),  w(1),
         w(2), 0,  -w(0),
        -w(1), w(0),  0;
    return W;
}

MatrixXd InverseKinematics::MatrixLog3(MatrixXd R) {
    double costh = (R.trace() - 1) / 2;
    MatrixXd W;
    VectorXd w;
    if (costh >= 1.0) {
        W = MatrixXd::Zero(3,3);
    } else if (costh <= -1.0) {
        if (abs(1 + R(2,2)) > 1e-8) {
            w = 1/sqrt(2 * (1 + R(2,2))) * R.col(2).array();
        } else if (abs(1 + R(1,1)) > 1e-8) {
            w = 1/sqrt(2 * (1 + R(1,1))) * R.col(1).array();
        } else if (abs(1 + R(0,0)) > 1e-8) {
            w = 1/sqrt(2 * (1 + R(0,0))) * R.col(0).array();
        }
        W = InverseKinematics::VecToSkewSymMat(M_PI * w.array());
    } else {
        double theta = acos(costh);
        W = theta / 2 / sin(theta) * (R - R.transpose());
    }
    return W;
};

VectorXd InverseKinematics::Se3ToVec(MatrixXd V_B) {
    VectorXd V_b(6);
    V_b << V_B(2,1), V_B(0,2), V_B(1,0),
            V_B(0,3), V_B(1,3), V_B(2,3);
    return V_b;
}

MatrixXd InverseKinematics::MatrixLog6(MatrixXd tf_mat) {
    MatrixXd R = tf_mat.block(0,0,3,3);
    VectorXd p = tf_mat.block(0,3,3,1);

    MatrixXd W = InverseKinematics::MatrixLog3(R);

    MatrixXd V_B(4,4);

    if (abs(W.minCoeff()) < 1e-8 && abs(W.maxCoeff()) < 1e-8) {
        V_B.block(0,3,3,1) = p;
    } else {
        double theta = acos((R.trace() - 1) / 2);
        double n1 = (1/theta - 1.0/2.0/tan(theta/2.0));
        VectorXd v = (MatrixXd::Identity(3,3) - 1.0/2.0 * W + n1 * W * W / theta) * p;

        V_B.block(0,0,3,3) = W;
        V_B.block(0,3,3,1) = v;
    }
    return V_B;
};

VectorXd InverseKinematics::f(VectorXd q) {
    VectorXd V(4);
    if (desired_config_type == "body_frame") {
        MatrixXd tf_body_ = InverseKinematics::TfBody(q);
        MatrixXd V_B = InverseKinematics::MatrixLog6(tf_body_);
        V = InverseKinematics::Se3ToVec(V_B);
    } else if (desired_config_type == "space_frame") {
        MatrixXd tf_body_ = InverseKinematics::TfSpace(q);
        MatrixXd V_B = InverseKinematics::MatrixLog6(tf_body_);
        VectorXd V_b = InverseKinematics::Se3ToVec(V_B);
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
