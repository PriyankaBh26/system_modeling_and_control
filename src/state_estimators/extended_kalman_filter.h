#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ExtendedKalmanFilter {
    public:
        ExtendedKalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd Q_in, MatrixXd R_in, double dt, int n_in, int m_in);

        // update state variables
        virtual VectorXd f(VectorXd x);

        // update outputs
        virtual VectorXd h(VectorXd x);

        // calculate f jacobian A
        virtual MatrixXd calculate_f_jacobian();

        // calculate h jacobian H 
        virtual MatrixXd calculate_h_jacobian();

        // predict state and error covariance
        void predict(MatrixXd A);

        // compute Kalman gain
        MatrixXd compute_kalman_gain(MatrixXd H);

        // compute the estimate
        VectorXd compute_estimate(VectorXd z);

        // update the error covariance
        void update(MatrixXd K, MatrixXd H);

    private:
        VectorXd x;
        MatrixXd P;
        MatrixXd Q;
        MatrixXd R;
        double dt;
        int n;
        int m;

};

#endif