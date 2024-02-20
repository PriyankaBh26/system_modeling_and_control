#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class PID {
    public:
        PID(int NUM_STATES);

        void SetGains(const MatrixXd& KP, const MatrixXd& KI, const MatrixXd& KD);

        VectorXd GenerateControlInput(VectorXd x_err, VectorXd dxdt_err);

        std::vector<VectorXd> GetXErrorHistory();

        std::vector<VectorXd> GetDxdtErrorHistory();

        std::vector<VectorXd> GetIntegralErrorHistory();

        std::vector<VectorXd> GetControlInputHistory();

        std::vector<std::string> GetColumnNames();

        friend std::ostream& operator<<(std::ostream& out, const PID& PID);

    private:
        int num_states;
        MatrixXd kp;
        MatrixXd ki;
        MatrixXd kd;
        std::vector<VectorXd> integral_err_history;
        std::vector<VectorXd> x_err_history;
        std::vector<VectorXd> dxdt_err_history;
        std::vector<VectorXd> control_input_history;

};


#endif