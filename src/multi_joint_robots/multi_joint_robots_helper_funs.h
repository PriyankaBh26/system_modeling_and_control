#ifndef MULTI_JOINT_ROBOT_HELPER_FUNS_H
#define MULTI_JOINT_ROBOT_HELPER_FUNS_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd TfmatInverse(MatrixXd Mat);

MatrixXd VecToSkewSymMat(VectorXd v);

VectorXd SkewSymMatToVec(MatrixXd W);

MatrixXd AdjointOfTfMatrix(MatrixXd tf_mat);

MatrixXd MatrixLog3(MatrixXd R);

MatrixXd MatrixLog6(MatrixXd tf_mat);

MatrixXd RotMatPosToTFMat(MatrixXd R, VectorXd p);

VectorXd Se3ToVec(MatrixXd V_B);

#endif