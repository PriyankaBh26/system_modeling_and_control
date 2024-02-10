# include <string>
# include <iostream>
# include <random>
# include <cmath>

# include <Eigen/Dense>

# include "controllers/model_reference_adaptive_controller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::LLT;

DirectMRAC::DirectMRAC(
    std::string str,
    int n1,
    int n2,
    int n3,
    VectorXd d0,
    double dt0,
    double gamma_w,
    double gamma_v,
    double gamma_kx,
    double gamma_kr,
    MatrixXd P_in,
    MatrixXd B_in) :   disturbance_model_feature_type(str),
                        num_states(n1),
                        num_features(n2),
                        num_outputs(n3),
                        disturbance_mean_std_bw(d0),
                        dt(dt0),
                        w(MatrixXd::Random(n2+1,n3)),
                        centers(n1+1,n2+1),
                        V(n1+1, n2+1),
                        Kx(n1), Kr(n3),
                        learning_rate_w(gamma_w),
                        learning_rate_v(gamma_v),
                        learning_rate_kx(gamma_kx),
                        learning_rate_kr(gamma_kr),
                        P(P_in),
                        B(B_in),
                        error(n1) {
    // Define a random number generator
    std::default_random_engine generator;

    std::normal_distribution<double> centers_distribution(disturbance_mean_std_bw(0), disturbance_mean_std_bw(1));
    std::normal_distribution<double> V_distribution(0, disturbance_mean_std_bw(2));
    
    for (int i{0}; i<num_states+1; i++){
        for (int j{0}; j<num_features+1; j++) {
            centers(i,j) = centers_distribution(generator);
            V(i,j) = abs(V_distribution(generator));
        }
    }
    // std::cout << "\n centers: \n" << centers;
};

VectorXd DirectMRAC::phi(VectorXd x_hat) {
    // radial basis function
    VectorXd phi_x(num_features+1);
    phi_x(0) = 1;
    for (int i{1}; i<num_features+1; i++) {   
        phi_x(i) = std::exp(-std::pow((centers.col(i) - x_hat).norm() / V(1,i), 2));
    }
    return phi_x;
};

VectorXd DirectMRAC::sigmoid(VectorXd x_hat) {
    // single hidden layer Neural Network with sigmoid activation fun
    VectorXd z = V.transpose() * x_hat;
    VectorXd a = centers.row(0);

    // size of sigmoid_x  = (num_features+1)
    // s_x = 1.0/(1.0 + exp(-a*z));
    VectorXd s_x = 1.0 /(1.0 + (-a.array() * z.array()).exp());
    s_x(0) = centers(0);
    return s_x;
};
        
MatrixXd DirectMRAC::dsigmoid(VectorXd x_hat) {
    // derivative of sigmoid activation fun
    // size of z  = (num_features+1)
    VectorXd z = V.transpose() * x_hat;
    // size of sigma_x  = (num_features+1)
    VectorXd s_x = DirectMRAC::sigmoid(x_hat);
    VectorXd ds_x = s_x.array() * (1 - s_x.array());
    ds_x(0) = 0;
    MatrixXd jac_s_x = ds_x.asDiagonal();    
    return jac_s_x;
};

void DirectMRAC::UpdateWeights(VectorXd x) {
    VectorXd x_hat(num_states+1);
    x_hat(0) = 1;
    x_hat.segment(1,num_states) = x;

    if (disturbance_model_feature_type == "radial_basis_fun") {
        // wdot size = (n2+1,1) * (1,n1) * (n1,n1) * (n1,1) = (n2+1,1)
        VectorXd wdot = learning_rate_w * phi(x_hat) * error.transpose() * P * B;
        w = w + wdot * dt;
    } else if (disturbance_model_feature_type == "single_hidden_layer_nn") {
        // wdot size = (n2+1,1 - (n2+1,n2+1) * (n2+1,n1+1) * (n1+1,1)) * (1,n1) * (n1,n1) * (n1,1) = (n2+1,1)
        MatrixXd wdot = -learning_rate_w * (sigmoid(x_hat) - dsigmoid(x_hat) * V.transpose() * x_hat) * error.transpose() * P * B;
        // Vdot size = (n1+1,1) * (1,n1) * (n1,n1) * (n1,1) * (1,n2+1) * (n2+1,n2+1) = (n1+1,n2+1)
        MatrixXd Vdot = -learning_rate_v * x_hat * error.transpose() * P * B * w.transpose() * dsigmoid(x_hat);
        w = w + wdot * dt;
        V = V + Vdot * dt;
    }
};

void DirectMRAC::UpdateKx(VectorXd x) {
    VectorXd Kxdot = learning_rate_kx * x * error.transpose() * P * B;
    Kx = Kx + Kxdot * dt;
};

void DirectMRAC::UpdateKr(VectorXd r) {
    VectorXd Krdot = learning_rate_kr * r * error.transpose() * P * B;
    Kr = Kr + Krdot * dt;
};

// void DirectMRAC::CalculateError(VectorXd x_ref, VectorXd x) {
//     error = x - x_ref;
// };

// VectorXd DirectMRAC::UpdateAdaptiveControlInput(VectorXd x) {
//     DirectMRAC::UpdateWeights(x);
//     VectorXd u_ad(1);
//     if (disturbance_model_feature_type == "radial_basis_fun") {
//         u_ad = w.transpose() * phi(x);
//     } else if (disturbance_model_feature_type == "single_hidden_layer_nn") {
//         u_ad = w.transpose() * sigmoid(V.transpose() * x);
//     }
//     return u_ad;
// };

// VectorXd DirectMRAC::UpdateControlInput(VectorXd r, VectorXd x_ref, VectorXd x) {
//     DirectMRAC::CalculateError(x_ref, x);

//     DirectMRAC::UpdateKx(x);
//     DirectMRAC::UpdateKr(r);

//     VectorXd u_ad = DirectMRAC::UpdateAdaptiveControlInput(x);
//     VectorXd u = - Kx * x + Kr * r - u_ad;
//     return u;
// };

MatrixXd DirectMRAC::GetW() {return w;};

MatrixXd DirectMRAC::GetV() {return V;};

VectorXd DirectMRAC::GetKx() {return Kx;};

VectorXd DirectMRAC::GetKr() {return Kr;};


std::ostream& operator << (std::ostream& out, DirectMRAC& system) {
    out << "\nMRAC parameters:\n";
    return out;
};
