# include <iostream>
# include <vector>
# include <Eigen/Dense>
# include "state_estimators/extended_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ExtendedKalmanFilter::ExtendedKalmanFilter(VectorXd x0, MatrixXd P0, 
                            MatrixXd Q_in, MatrixXd R_in, 
                            double dt, int n_in, int m_in) : x(x0), P(P0), 
                                                             Q(Q_in), R(R_in), 
                                                             dt(dt), n(n_in), m(m_in) {};

// predict state and error covariance
void ExtendedKalmanFilter::Predict(MatrixXd A, VectorXd Budt) {
    x = f(Budt);
    P = A * P * A.transpose() + Q;
}
// compute Kalman gain
MatrixXd ExtendedKalmanFilter::ComputeKalmanGain(MatrixXd H) {
    MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    return K;
}
// compute the estimate
VectorXd ExtendedKalmanFilter::ComputeEstimate(VectorXd z, VectorXd Budt) {
    MatrixXd A = CalculateFxJacobian();
    MatrixXd H = CalculateHxJacobian();
    Predict(A, Budt);
    MatrixXd K = ComputeKalmanGain(H);
    Update(K, H, z);
    return x;
}
// update the error covariance
void ExtendedKalmanFilter::Update(MatrixXd K, MatrixXd H, VectorXd z) {
    x = x + K * (z - h());
    P = P - K * H * P;
}

// getter functions
int ExtendedKalmanFilter::GetN() {return n;}

int ExtendedKalmanFilter::GetM() {return m;}

double ExtendedKalmanFilter::GetDt() {return dt;}

VectorXd ExtendedKalmanFilter::GetX() {return x;}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}