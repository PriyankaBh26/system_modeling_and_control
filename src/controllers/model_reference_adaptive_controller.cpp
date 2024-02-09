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
    int features,
    VectorXd d0,
    int n,
    double dt0,
    double gamma_w,
    double gamma_v,
    double gamma_kx,
    double gamma_kr,
    MatrixXd A_ref_in,
    MatrixXd B_in) :   disturbance_model_feature_type(str),
                        num_features(features),
                        disturbance_mean_std_bw(d0),
                        num_states(n),
                        dt(dt0),
                        w(VectorXd::Random(n)),
                        centers(features),
                        V(features),
                        Kx(n), Kr(n),
                        learning_rate_w(gamma_w),
                        learning_rate_v(gamma_v),
                        learning_rate_kx(gamma_kx),
                        learning_rate_kr(gamma_kr),
                        P(n,n),
                        A_ref(A_ref_in),
                        B(B_in),
                        error(n) {
    DirectMRAC::CalculateMatrixP();
    // Define a random number generator
    std::default_random_engine generator;

    std::normal_distribution<double> centers_distribution(disturbance_mean_std_bw(0), disturbance_mean_std_bw(1));
    std::normal_distribution<double> V_distribution(0, disturbance_mean_std_bw(2));

    for (int i{0}; i<num_features; i++) {
        centers(i) = centers_distribution(generator);
        V(i) = abs(V_distribution(generator));
    }
};

VectorXd DirectMRAC::phi(VectorXd x) {
    // radial basis function
    VectorXd phi_x(num_states);
    for (int i{0}; i<num_states; i++) {    
        VectorXd phi_x_f = (-((centers.array() - x(i)) * V.array().inverse()).square()).exp();
        phi_x(i) = phi_x_f.sum();
    }
return phi_x;
};

VectorXd DirectMRAC::sigmoid(VectorXd Y) {
    // single hidden layer Neural Network with sigmoid activation fun
    VectorXd s_x(num_features);
    for (int i{0}; i<num_features; i++) {
        s_x(i) = 1/(1 + exp(-Y(i)));
    }
    return s_x;
};
        
VectorXd DirectMRAC::dsigmoid(VectorXd Y) {
    // derivative of sigmoid activation fun
    VectorXd ds_x(num_features);
    for (int i{0}; i<num_features; i++) {
        double s_x = 1/(1 + exp(-Y(i)));
        ds_x(i) = s_x * (1 - s_x);
    }
    return ds_x;
};

void DirectMRAC::UpdateWeights(VectorXd x) {
    if (disturbance_model_feature_type == "radial_basis_fun") {
        VectorXd wdot = learning_rate_w * phi(x) * error.transpose() * P * B;
        w = w + wdot * dt;
    } else if (disturbance_model_feature_type == "single_hidden_layer_nn") {
        VectorXd wdot = -(sigmoid(V.transpose() * x) - dsigmoid(V.transpose() * x) * V.transpose() * x) * error.transpose() * P * B * learning_rate_w;
        VectorXd Vdot = -learning_rate_v * x * error.transpose() * P * B * w.transpose() * sigmoid(V.transpose() * x);
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

void DirectMRAC::CalculateMatrixP() {
    // Solve the Lyapunov equation AmP + PAm^T = -Q
    MatrixXd Q = MatrixXd::Identity(num_states, num_states);
    LLT<MatrixXd> lltOfA(A_ref.transpose());
    if (lltOfA.info() == Eigen::Success) {
        // A is positive definite
        P = lltOfA.solve(-Q);
        P = (P + P.transpose()) / 2; // Ensure symmetric solution
    } else {
        // A is not positive definite
        std::cout << "Matrix A is not positive definite. Cannot solve Lyapunov equation." << std::endl;
    }

};

void DirectMRAC::CalculateError(VectorXd x_ref, VectorXd x) {
    error = x - x_ref;
};

VectorXd DirectMRAC::UpdateAdaptiveControlInput(VectorXd x) {
    DirectMRAC::UpdateWeights(x);
    VectorXd u_ad(1);
    if (disturbance_model_feature_type == "radial_basis_fun") {
        u_ad = w.transpose() * phi(x);
    } else if (disturbance_model_feature_type == "single_hidden_layer_nn") {
        u_ad = w.transpose() * sigmoid(V.transpose() * x);
    }
    return u_ad;
};

VectorXd DirectMRAC::UpdateControlInput(VectorXd r, VectorXd x_ref, VectorXd x) {
    DirectMRAC::CalculateError(x_ref, x);

    DirectMRAC::UpdateKx(x);
    DirectMRAC::UpdateKr(r);

    VectorXd u_ad = DirectMRAC::UpdateAdaptiveControlInput(x);
    VectorXd u = - Kx * x + Kr * r - u_ad;
    return u;
};

std::ostream& operator << (std::ostream& out, DirectMRAC& system) {
    out << "MRAC parameters:\n";
    return out;
};
