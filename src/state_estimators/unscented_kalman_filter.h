#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UnscentedKalmanFilter {
    public:
        UnscentedKalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd Q, 
                              MatrixXd R, double dt, int n, int m, double kappa);

        virtual ~UnscentedKalmanFilter();

        // update state variables
        virtual VectorXd f(VectorXd Y, VectorXd Bu);

        // update outputs
        virtual VectorXd h(VectorXd Y);

        // find Xi and W
        MatrixXd ComputeSigmaPointsXi();
        void ComputeWeights();

        // predict state and error covariance
        VectorXd PredictMean(MatrixXd Xi);
        MatrixXd PredictCovariance(VectorXd x_mean, MatrixXd Xi, MatrixXd err_cov);

        // compute Kalman gain
        MatrixXd ComputeKalmanGain(MatrixXd Pz, MatrixXd f_xi, 
                                    MatrixXd h_xi, VectorXd z_cap);

        // compute the estimate
        VectorXd ComputeEstimate(VectorXd z, VectorXd Bu);

        // getter functions
        int GetN();

        int GetM();

        double GetDt();

    private:
        VectorXd x;
        MatrixXd P;
        MatrixXd Q;
        MatrixXd R;
        double dt;
        int n;
        int m;
        double kappa;
        VectorXd W;

};

#endif