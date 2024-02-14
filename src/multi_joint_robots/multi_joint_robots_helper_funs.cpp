
# include <Eigen/Dense>

# include "multi_joint_robots/multi_joint_robots_helper_funs.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;


MatrixXd TfmatInverse(MatrixXd Mat) {
    MatrixXd Mat_inverse(4,4);
    MatrixXd R = Mat.block(0,0,3,3);
    VectorXd p = Mat.block(0,3,3,1);

    Mat_inverse.block(0,0,3,3) = R.transpose();
    Mat_inverse.block(0,3,3,1) = -R.transpose() * p;
    Mat_inverse(3,3) = 1;
    return Mat_inverse;
};

MatrixXd VecToSkewSymMat(VectorXd v) {
    MatrixXd skew_sym_matrix(3,3);
    skew_sym_matrix << 0, -v(2), v(1),
                        v(2), 0, -v(0),
                        -v(1), v(0), 0;

    return skew_sym_matrix;
};

VectorXd SkewSymMatToVec(MatrixXd W) {
    VectorXd w_theta(3);
    w_theta << W(2,1), W(0,2), W(1,0);
    return w_theta;
};

MatrixXd AdjointOfTfMatrix(MatrixXd tf_mat) {
    MatrixXd ad_tf_mat(6,6);
    MatrixXd R(3,3);
    R = tf_mat.block(0,0,3,3);

    VectorXd p(3);
    p << tf_mat(0,3), tf_mat(1,3), tf_mat(2,3);

    MatrixXd p_mat = VecToSkewSymMat(p);

    ad_tf_mat.block(0,0,3,3) = R;
    ad_tf_mat.block(3,3,3,3) = R;
    ad_tf_mat.block(3,0,3,3) = p_mat * R;
    return ad_tf_mat;
};

MatrixXd MatrixExp3(MatrixXd W, double theta) {
    MatrixXd exp_mat = MatrixXd::Identity(3,3) + sin(theta) / theta  * W + (1 - cos(theta)) / theta / theta * W * W;
    return exp_mat;
}

MatrixXd MatrixLog3(MatrixXd R) {
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
        W = VecToSkewSymMat(M_PI * w.array());
    } else {
        double theta = acos(costh);
        W = theta / 2 / sin(theta) * (R - R.transpose());
    }
    return W;
};

MatrixXd MatrixLog6(MatrixXd tf_mat) {
    MatrixXd R = tf_mat.block(0,0,3,3);
    VectorXd p = tf_mat.block(0,3,3,1);

    MatrixXd W = MatrixLog3(R);

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

MatrixXd RotMatPosToTFMat(MatrixXd R, VectorXd p) {
    MatrixXd TFMat(4,4);
    TFMat.block(0,0,3,3) = R;
    TFMat.block(0,3,3,1) = p;
    TFMat(3,3) = 1;
    return TFMat;
};

VectorXd Se3ToVec(MatrixXd V_B) {
    VectorXd V_b(6);
    V_b << V_B(2,1), V_B(0,2), V_B(1,0),
            V_B(0,3), V_B(1,3), V_B(2,3);
    return V_b;
};

MatrixXd VecToSe3(VectorXd V_b) {
    MatrixXd V_mat(4,4);

    VectorXd w(3);
    w << V_b(0), V_b(1), V_b(2);

    VectorXd v(3);
    v << V_b(3), V_b(4), V_b(5);

    V_mat.block(0,0,3,3) = VecToSkewSymMat(w);
    V_mat.block(0,3,3,1) = v;

    return V_mat;
};
