#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ExtendedKalmanFilter {
    public:
        ExtendedKalmanFilter(VectorXd x0, MatrixXd P0, 
                             MatrixXd Q_in, MatrixXd R_in, 
                             double dt, int n_in, int m_in);

        virtual ~ExtendedKalmanFilter();

        // update state variables
        virtual VectorXd f(VectorXd Budt);

        // update outputs
        virtual VectorXd h();

        // calculate f jacobian A
        virtual MatrixXd CalculateFxJacobian();

        // calculate h jacobian H 
        virtual MatrixXd CalculateHxJacobian();

        // predict state and error covariance
        void Predict(MatrixXd A, VectorXd Budt);

        // compute Kalman gain
        MatrixXd ComputeKalmanGain(MatrixXd H);

        // compute the estimate
        VectorXd ComputeEstimate(VectorXd z, VectorXd Budt);

        // update the error covariance
        void Update(MatrixXd K, MatrixXd H, VectorXd z);

        // getter functions
        int GetN();

        int GetM();

        double GetDt();

        VectorXd GetX();

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