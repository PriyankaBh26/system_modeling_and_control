#ifndef MODEL_REFERENCE_ADAPTIVE_CONTROLLER_H
#define MODEL_REFERENCE_ADAPTIVE_CONTROLLER_H

# include <string>
# include <iostream>

# include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;

class DirectMRAC {
    public:
        DirectMRAC(std::string disturbance_model_feature_type,
                    int num_features,
                    VectorXd disturbance_mean_std_bw,
                    int num_states,
                    double dt,
                    VectorXd w0,
                    double learning_rate_w,
                    double learning_rate_v,
                    double learning_rate_kx,
                    double learning_rate_kr
                    MatrixXd A_ref,
                    MatrixXd B);

        VectorXd phi(VectorXd x);

        VectorXd sigmoid(VectorXd Y);
        
        VectorXd dsigmoid(VectorXd Y);

        void UpdateWeights(VectorXd x);

        void UpdateKx(VectorXd x);

        void UpdateKr(VectorXd r);

        void CalculateError(VectorXd x_ref, VectorXd x);

        void CalculateMatrixP();

        VectorXd UpdateAdaptiveControlInput(VectorXd x);

        VectorXd UpdateControlInput(VectorXd r, VectorXd x_ref, VectorXd x);

        friend std::ostream& operator << (std::ostream& out, DirectMRAC& system);

    private:
        std::string disturbance_model_feature_type;
        int num_features;
        VectorXd disturbance_mean_std_bw;
        VectorXd centers;
        VectorXd V;
        int num_states;
        double dt;
        VectorXd w;
        double learning_rate_w;
        double learning_rate_v;
        double learning_rate_kx;
        double learning_rate_kr;
        MatrixXd A_ref;
        MatrixXd B;
        VectorXd error;
};

#endif