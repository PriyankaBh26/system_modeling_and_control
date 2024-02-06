# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "unscented_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::LLT;

UnscentedKalmanFilter::UnscentedKalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd Q, 
                        MatrixXd R, int n, int m, double kappa) : x(x0), P(P0), Q(Q), R(R), n(n), 
                                                                  m(m), kappa(kappa), W(2*n+1);

// update state variables
virtual VectorXd UnscentedKalmanFilter::f(VectorXd x) {VectorXd x(n)};

// update outputs
virtual VectorXd UnscentedKalmanFilter::h(vectorXd x) {VectorXd z(m)};

// compute Xi
MatrixXd UnscentedKalmanFilter::sigma_points_Xi() {
    MatrixXd Xi(n, 2*n+1);
    Xi.col(0) = x;

    LLT<MatrixXd> lltOfA((n + kappa) * P); // compute the Cholesky decomposition of A
    MatrixXd U = lltOfA.matrixL(); // retrieve factor L  in the decomposition
    // The previous two lines can also be written as "U = A.llt().matrixL()"

    for (int i{1}; i<=n; i++) {
        Xi.col(i) = Xi.col(0) + U.row(i-1).transpose();
        Xi.col(n+i) = Xi.col(0) - U.row(i-1).transpose();
    }
    return Xi;
};

// compute W
void UnscentedKalmanFilter::compute_weights() {
    VectorXd W(2*n+1);
    W(0) = kappa / (n + kappa);

    for (int i{1}; i<=n; i++) {
        W(i) = 1 / (2 * (n + kappa));
        W(n+i) = 1 / (2 * (n + kappa));
    }
};

// perform unscented transform to predict state mean 
VectorXd UnscentedKalmanFilter::predict_mean(VectorXd Xi) {
    // compute mean x
    VectorXd xm(n);
    for (int i{0}; i<Xi.cols(); i++) {
        xm = xm + W(i) * Xi.col(i);
    }
    return xm;
};

// perform unscented transform to predict state covariance 
MatrixXd UnscentedKalmanFilter::predict_covariance(VectorXd Xi, MatrixXd err_cov) {
    // compute covariance
    MatrixXd xcov(n,n);
    for (int i{0}; i<Xi.cols(); i++) {
        xcov = xcov + W(i) * (Xi.col(i) - xm) * (Xi.col(i) - xm).transpose();
    }
    xcov + err_cov;
    return xcov;
};


// compute Kalman gain
MatrixXd UnscentedKalmanFilter::compute_kalman_gain(MatrixXd Pz, VectorXd f_xi, vectorxd h_xi, vectorXd z_cap) {
    MatrixXd Pxz(n,m);
    for (int i{0}; i<2*n+1; i++) {
        Pxz = Pxz + W(i) * (f_xi - x) * (h_xi - z_cap).transpose();
    }
    K = Pxz * Pz.inverse();
    return K;
};

// compute the estimate
VectorXd UnscentedKalmanFilter::compute_estimate(VectorXd z) {
    UnscentedKalmanFilter::compute_weights();
    MatrixXd Xi = UnscentedKalmanFilter::sigma_points_Xi();

    // predict state
    VectorXd f_xi = f(Xi);
    x = UnscentedKalmanFilter::predict_mean(f_xi);
    // predict state covariance
    P = UnscentedKalmanFilter::predict_covariance(f_xi, Q);

    // predict measurement
    VectorXd h_xi = h(Xi);
    VectorXd z_cap = UnscentedKalmanFilter::predict_mean(h_xi);
    // predict measurement covariance
    MatrixXd Pz = UnscentedKalmanFilter::predict_covariance(h_xi, R);

    // compute kalman gain
    MatrixXd K = UnscentedKalmanFilter::compute_kalman_gain(Pz, f_xi, h_xi, z_cap);

    // update the state estimate
    x = x + K * (z - z_cap);

    // update the error covariance
    P = P - K * Pz * K.transpose();

    return x;
};

UnscentedKalmanFilter::~UnscentedKalmanFilter() {}