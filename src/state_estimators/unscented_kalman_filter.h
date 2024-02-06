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
        virtual VectorXd f(VectorXd Y);

        // update outputs
        virtual VectorXd h(VectorXd Y);

        // find Xi and W
        MatrixXd sigma_points_Xi();
        void compute_weights();

        // predict state and error covariance
        VectorXd predict_mean(MatrixXd Xi);
        MatrixXd predict_covariance(VectorXd x_mean, MatrixXd Xi, MatrixXd err_cov);

        // compute Kalman gain
        MatrixXd compute_kalman_gain(MatrixXd Pz, MatrixXd f_xi, 
                                    MatrixXd h_xi, VectorXd z_cap);

        // compute the estimate
        VectorXd compute_estimate(VectorXd z);

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