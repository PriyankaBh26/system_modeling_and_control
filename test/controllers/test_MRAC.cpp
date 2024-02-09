# include <string>
# include <iostream>
# include <random>

# include <Eigen/Dense>

# include "controllers/model_reference_adaptive_controller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {

    // reference model matrices A_ref and B_ref
    MatrixXd A_ref(num_states, num_states);
    A_ref << 0 , 1,
            -4, -2;
    // Compute the eigenvalues of A_ref
    Eigen::EigenSolver<MatrixXd> solver(A_ref);
    Eigen::VectorXcd eigenvalues = solver.eigenvalues();
    std::cout << "A_ref eigenvalues:\n"  << eigenvalues;

    MatrixXd B_ref(1, num_states);
    B_ref << 0, 4;

    // nominal plant model matrices A and B
    MatrixXd A(num_states, num_states);
    A_ << 0, 1,
          0, 0;
    MatrixXd B(1, num_states);
    B << 0, 1;

    // solved Lyapunov equation using controller helper funs and A_ref
    MatrixXd P(num_states, num_states);
    P << 1.5, 0.125,
         0.125, 0.3125;

    // initialize MRAC model
    std::string disturbance_model_feature_type = "radial_basis_fun";
    int num_features = 10;
    VectorXd disturbance_mean_std_bw(3);
    disturbance_mean_std_bw << 1,2,3;
    int num_states = 2;
    double dt = 0.01;
    double learning_rate_w = 10;
    double learning_rate_v = 10;
    double learning_rate_kx = 10;
    double learning_rate_kr = 10;
    
    DirectMRAC* mrac = new DirectMRAC(disturbance_model_feature_type,
                                      num_features,
                                      disturbance_mean_std_bw,
                                      num_states,
                                      dt,
                                      learning_rate_w,
                                      learning_rate_v,
                                      learning_rate_kx,
                                      learning_rate_kr,
                                      P,
                                      B);

    std::cout << *mrac;


    delete mrac;


    return 0;
}
