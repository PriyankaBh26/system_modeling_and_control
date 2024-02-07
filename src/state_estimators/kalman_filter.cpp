# include <iostream>
# include <vector>
# include <Eigen/Dense>
# include "state_estimators/kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd A_in, 
        MatrixXd Q_in, MatrixXd H_in, MatrixXd R_in) : x(x0), P(P0), A(A_in), Q(Q_in), H(H_in), R(R_in) {};

// predict state and error covariance
void KalmanFilter::Predict() {
    x = A * x;
    P = A * P * A.transpose() + Q;
}
// compute Kalman gain
MatrixXd KalmanFilter::ComputeKalmanGain() {
    MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    return K;
}
// compute the estimate
VectorXd KalmanFilter::ComputeEstimate(VectorXd z) {
    Predict();
    MatrixXd K = ComputeKalmanGain();
    Update(K, z);
    return x;
}
// update the error covariance
void KalmanFilter::Update(MatrixXd K, VectorXd z) {
    x = x + K * (z - H * x);
    P = P - K * H * P;
}
