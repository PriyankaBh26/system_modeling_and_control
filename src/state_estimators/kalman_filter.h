#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
    public:
        KalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd A_in, MatrixXd Q_in, MatrixXd H_in, MatrixXd R_in);

        // predict state and error covariance
        void predict();

        // compute Kalman gain
        MatrixXd compute_kalman_gain();

        // compute the estimate
        VectorXd compute_estimate(VectorXd z);

        // update the error covariance
        void update(MatrixXd K);

    private:
        VectorXd x;
        MatrixXd P;
        MatrixXd A;
        MatrixXd Q;
        MatrixXd H;
        MatrixXd R;

};

#endif