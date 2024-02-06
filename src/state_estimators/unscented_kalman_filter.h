#ifndef UNSCENTED_KALMAN_FILTER_H
#define UNSCENTED_KALMAN_FILTER_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UnscentedKalmanFilter {
    public:
        UnscentedKalmanFilter(VectorXd x0, MatrixXd P0, MatrixXd Q, 
                              MatrixXd R, int n, int m, double kappa);

        virtual ~UnscentedKalmanFilter();

        // update state variables
        virtual VectorXd f(VectorXd x);

        // update outputs
        virtual VectorXd h(vectorXd x);

        // find Xi and W
        MatrixXd sigma_points_Xi();
        void compute_weights();

        // predict state and error covariance
        VectorXd predict_mean(VectorXd Xi);
        MatrixXd predict_covariance(VectorXd Xi, MatrixXd err_cov);

        // compute Kalman gain
        MatrixXd compute_kalman_gain(MatrixXd H);

        // compute the estimate
        VectorXd compute_estimate(VectorXd z);

    private:
        VectorXd x;
        MatrixXd P;
        MatrixXd Q;
        MatrixXd R;
        int n;
        int m;
        double kappa;
        VectorXd W;

};

#endif