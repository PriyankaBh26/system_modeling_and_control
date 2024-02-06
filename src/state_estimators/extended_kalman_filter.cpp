# include <iostream>
# include <vector>
# include <Eigen/Dense>
# include "state_estimators/extended_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ExtendedKalmanFilter::ExtendedKalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd Q_in, 
                             MatrixXd R_in, double dt, int n_in, int m_in) : x(x0), P(P0), Q(Q_in), 
                                                                            R(R_in), dt(dt), n(n_in), m(m_in) {};

// update state variables
VectorXd ExtendedKalmanFilter::f() {
    VectorXd x(n); 
    return x;
};

// update outputs
VectorXd ExtendedKalmanFilter::h() {
    VectorXd y(n);
    return y;
};

// calculate f jacobian A
MatrixXd ExtendedKalmanFilter::calculate_f_jacobian() {
    MatrixXd A(n, n);
    return A;
};

// calculate h jacobian H 
MatrixXd ExtendedKalmanFilter::calculate_h_jacobian() {
    MatrixXd H(m, n);
    return H;
};

// predict state and error covariance
void ExtendedKalmanFilter::predict(MatrixXd A) {
    x = f();
    P = A * P * A.transpose() + Q;
}
// compute Kalman gain
MatrixXd ExtendedKalmanFilter::compute_kalman_gain(MatrixXd H) {
    MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    return K;
}
// compute the estimate
VectorXd ExtendedKalmanFilter::compute_estimate(VectorXd z) {
    MatrixXd A = calculate_f_jacobian();
    MatrixXd H = calculate_h_jacobian();
    predict(A);
    MatrixXd K = compute_kalman_gain(H);
    update(K, H, z);
    return x;
}
// update the error covariance
void ExtendedKalmanFilter::update(MatrixXd K, MatrixXd H, VectorXd z) {
    x = x + K * (z - h());
    P = P - K * H * P;
}

// getter functions
int ExtendedKalmanFilter::GetN() {return n;}

int ExtendedKalmanFilter::GetM() {return m;}

double ExtendedKalmanFilter::GetDt() {return dt;}

VectorXd ExtendedKalmanFilter::GetX() {return x;}

ExtendedKalmanFilter::~ExtendedKalmanFilter() {}