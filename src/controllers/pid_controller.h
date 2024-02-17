#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class PID {
    public:
        PID(int NUM_STATES);

        void SetGains(const MatrixXd& KP, const MatrixXd& KI, const MatrixXd& KD);

        void CalculateError(VectorXd x_ref, VectorXd x);

        std::vector<VectorXd> GetErrorHistory();

        std::vector<VectorXd> GetControlInputHistory();

        VectorXd GenerateControlInput();

        std::vector<std::string> GetColumnNames();

        friend std::ostream& operator<<(std::ostream& out, const PID& PID);

    private:
        MatrixXd kp;
        MatrixXd ki;
        MatrixXd kd;
        VectorXd error;
        VectorXd d_error;
        VectorXd sum;
        VectorXd previous_error;
        std::vector<VectorXd> error_history;
        std::vector<VectorXd> control_input_history;

};


#endif