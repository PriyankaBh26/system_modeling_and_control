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

InverseKinematics::InverseKinematics(int num_joints, 
                          std::vector<std::string> joint_type, 
                          MatrixXd tf_home, 
                          MatrixXd screw_space, 
                          MatrixXd screw_body,
                          std::string str,
                          MatrixXd tf_d,
                          VectorXd q0,
                          double tolerance,
                          int max_iterations) : ForwardKinematics(num_joints, 
                                                            joint_type,
                                                            tf_home,
                                                            screw_space,
                                                            screw_body), 

                                                NewtonRaphson(num_joints,
                                                                6,
                                                                q0,
                                                                tolerance,
                                                                max_iterations),
                                                                desired_config_type(str),
                                                                tf_desired(tf_d),
                                                                q(q0) {};


MatrixXd InverseKinematics::TfSpaceToBody(MatrixXd tf_desired, VectorXd q) {
    TfInSpaceFrame(q);
    MatrixXd tf_mat_sb = GetTfSpace();
    tf_body = tf_mat_sb.inverse() * tf_desired;
    return tf_body;
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

// MatrixXd InverseKinematics::MatrixLog3(MatrixXd R) {
//     double costh = (R.trace() - 1) / 2;
//     if (costh >= 1.0) {
//         MatrixXd W = MatrixXd::zero(3,3);
//     } else if (costh <= -1.0) {
//         if (abs(1 + R(2,2)) > 1e-8) {
//             VecrorXd w = 1/sqrt(2 * (1 + R(2,2))) * R.col(2).array();
//         } else if (abs(1 + R(1,1)) > 1e-8) {
//             VecrorXd w = 1/sqrt(2 * (1 + R(1,1))) * R.col(1).array();
//         } else if (abs(1 + R(0,0)) > 1e-8) {
//             VecrorXd w = 1/sqrt(2 * (1 + R(0,0))) * R.col(0).array();
//         }
//         MatrixXd W = InverseKinematics::VecToSkewSymMat(M_PI * w);
//     } else {
//         double theta = acos(costh);
//         MatrixXd W = theta / 2 / sin(theta) * (R - R.transpose());
//     }
//     return W;
// };

// VectorXd InverseKinematics::Se3ToVec(MatrixXd V_B) {
//     VectorXd V_b(6);
//     V_b << V_B(2,1), V_B(0,2), V_B(1,0),
//             V_B(0,3), V_B(1,3), V_B(2,3);
//     return V_b;
// }

// VectorXd InverseKinematics::BodyTwistFromTF(MatrixXd tf_body) {
//     MatrixXd R = tf_body.block(0,0,3,3);
//     VectorXd p = tf_body.block(0,3,3,1);

//     MatrixXd W = InverseKinematics::MatrixLog3(R);

//     Matrix V_B(4,4);

//     if (abs(R.minCoeff()) < 1e-8 && abs(R.maxCoeff()) < 1e-8) {
//         V_B.block(0,3,3,1) = p;
//         double theta = p.norm();
//         VectorXd w = vectorXd::zero(3);
//         VectorXd v = p/p.norm();
//     } else {
//         double theta = acos((R.trace() - 1) / 2);
//         VectorXd w = InverseKinematics::SkewSymMatToVec(W / theta);
//         VectorXd v = (1/theta * MatrixXd::Identity(3,3) - 1/2 * W + (1/theta - 1/2/tan(theta/2)) * W * W) * p;
//         V_B.block(0,0,3,3) = W;
//         V_B.block(0,3,3,1) = v;

//     }
//     VectorXd V_b = InverseKinematics::Se3ToVec(V_B);
//     return V_b;
// };

// VectorXd InverseKinematics::f(VectorXd q) {
//     MatrixXd tf_body = InverseKinematics::TfSpaceToBody(tf_desired, q);

//     return v;
// };

// MatrixXd InverseKinematics::dfdq(VectorXd q) {
//     MatrixXd mat = SpaceJacobian(q);
//     return mat;
// };

// VectorXd InverseKinematics::SolveIK() {
//     Iterate();
//     std::vector<VectorXd> q_history = GetSolutionHistory();
//     std::cout << "Solution, q =\n" << q_history.back() << "\n";
//     return q_history.back();
// };

std::ostream& operator << (std::ostream& out, InverseKinematics& system) {
    out << "\n Inverse Kinematics parameters :\n";
    return out;
};
