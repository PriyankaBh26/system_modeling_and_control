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
                    int num_states,
                    int num_outputs,
                    VectorXd disturbance_mean_std_bw,
                    double dt,
                    double learning_rate_w,
                    double learning_rate_v,
                    double learning_rate_kx,
                    double learning_rate_kr,
                    MatrixXd P,
                    MatrixXd B);

        VectorXd phi(VectorXd x);

        VectorXd sigmoid(VectorXd x);
        
        MatrixXd dsigmoid(VectorXd x);

        VectorXd CalculateXhat(VectorXd x);

        void UpdateWeights(VectorXd x);

        void UpdateKx(VectorXd x);

        void UpdateKr(VectorXd r);

        void CalculateError(VectorXd x_ref, VectorXd x);

        VectorXd UpdateAdaptiveControlInput(VectorXd x);

        VectorXd UpdateControlInput(VectorXd r, VectorXd x_ref, VectorXd x);

        MatrixXd GetW();

        MatrixXd GetV();

        VectorXd GetKx();

        VectorXd GetKr();

        VectorXd GetError();

        std::vector<VectorXd> GetErrorHistory();

        std::vector<VectorXd> GetControlInputHistory();

        friend std::ostream& operator << (std::ostream& out, DirectMRAC& system);

    private:
        std::string disturbance_model_feature_type;
        int num_states;
        int num_features;
        int num_outputs;
        VectorXd disturbance_mean_std_bw;
        double dt;
        MatrixXd w;
        MatrixXd centers;
        MatrixXd V;
        VectorXd Kx;
        VectorXd Kr;
        double learning_rate_w;
        double learning_rate_v;
        double learning_rate_kx;
        double learning_rate_kr;
        MatrixXd P;
        MatrixXd B;
        VectorXd error;
};

#endif