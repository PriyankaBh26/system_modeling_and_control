# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "kalman_filter.h"
# include "extended_kalman_filter.h"
# include "unscented_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


int main() {
    std::string chosen_kalman_filter = "kalman_filter";

    if (chosen_kalman_filter == "kalman_filter") {
        std::unique_ptr<KalmanFilter> KF (new KalmanFilter(x0, P0, A, Q, H, R));

        VectorXd x = KF->compute_estimate(z);
    }


}