# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "state_estimators/unscented_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::LLT;

UnscentedKalmanFilter::UnscentedKalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd Q, 
                                             MatrixXd R, double dt, int n, 
                                             int m, double kappa) : x(x0), P(P0), Q(Q), 
                                                                    R(R), dt(dt), n(n), 
                                                                    m(m), kappa(kappa), W(2*n+1) {
                                                                          UnscentedKalmanFilter::ComputeWeights();};

// update state variables
VectorXd UnscentedKalmanFilter::f(VectorXd Y) {
    VectorXd YY(n);
    return YY;
};

// update outputs
VectorXd UnscentedKalmanFilter::h(VectorXd Y) {
    VectorXd YY(m);
    return YY;
};

// compute Xi
MatrixXd UnscentedKalmanFilter::ComputeSigmaPointsXi() {
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
void UnscentedKalmanFilter::ComputeWeights() {
    W(0) = kappa / (n + kappa);

    for (int i{1}; i<=n; i++) {
        W(i) = 1 / (2 * (n + kappa));
        W(n+i) = 1 / (2 * (n + kappa));
    }
};

// perform unscented transform to predict state mean 
VectorXd UnscentedKalmanFilter::PredictMean(MatrixXd Xi) {
    // compute mean x
    VectorXd xm(Xi.rows());
    for (int i{0}; i<Xi.cols(); i++) {
        xm = xm + W(i) * Xi.col(i);
    }
    return xm;
};

// perform unscented transform to predict state covariance 
MatrixXd UnscentedKalmanFilter::PredictCovariance(VectorXd xm, MatrixXd Xi, MatrixXd err_cov) {
    // compute covariance
    MatrixXd xcov(err_cov.rows(), err_cov.cols());
    for (int i{0}; i<Xi.cols(); i++) {
        xcov = xcov + W(i) * (Xi.col(i) - xm) * (Xi.col(i) - xm).transpose();
    }
    xcov += err_cov;
    return xcov;
};


// compute Kalman gain
MatrixXd UnscentedKalmanFilter::ComputeKalmanGain(MatrixXd Pz, MatrixXd f_xi, 
                                                    MatrixXd h_xi, VectorXd z_cap) {
    MatrixXd Pxz(n,m);
    for (int i{0}; i<2*n+1; i++) {
        Pxz = Pxz + W(i) * (f_xi.col(i) - x) * (h_xi.col(i) - z_cap).transpose();
    }
    MatrixXd K = Pxz * Pz.inverse();
    return K;
};

// compute the estimate
VectorXd UnscentedKalmanFilter::ComputeEstimate(VectorXd z) {
    MatrixXd Xi = UnscentedKalmanFilter::ComputeSigmaPointsXi();

    // predict state
    MatrixXd f_xi(n, 2*n+1);
    for (int i{0}; i<Xi.cols(); i++) {
        f_xi.col(i) = f(Xi.col(i));
    }
    x = UnscentedKalmanFilter::PredictMean(f_xi);
    // predict state covariance
    P = UnscentedKalmanFilter::PredictCovariance(x, f_xi, Q);

    // predict measurement
    MatrixXd h_xi(m, 2*n+1);
    for (int i{0}; i<Xi.cols(); i++) {
        h_xi.col(i) = h(Xi.col(i));
    }
    VectorXd z_cap = UnscentedKalmanFilter::PredictMean(h_xi);
    // predict measurement covariance
    MatrixXd Pz = UnscentedKalmanFilter::PredictCovariance(z_cap, h_xi, R);

    // compute kalman gain
    MatrixXd K = UnscentedKalmanFilter::ComputeKalmanGain(Pz, f_xi, h_xi, z_cap);

    // update the state estimate
    x = x + K * (z - z_cap);

    // update the error covariance
    P = P - K * Pz * K.transpose();

    return x;
};

// getter functions
int UnscentedKalmanFilter::GetN() {return n;}

int UnscentedKalmanFilter::GetM() {return m;}

double UnscentedKalmanFilter::GetDt() {return dt;}

UnscentedKalmanFilter::~UnscentedKalmanFilter() {}