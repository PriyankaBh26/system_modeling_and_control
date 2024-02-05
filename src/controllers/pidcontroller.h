#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

# include <Eigen/Dense>

class PID {
    public:
        PID(int NUM_STATES);

        void SetGains(const Eigen::MatrixXd& KP, const Eigen::MatrixXd& KI, const Eigen::MatrixXd& KD);

        void CalculateError(Eigen::VectorXd x_ref, Eigen::VectorXd x);

        std::vector<Eigen::VectorXd> GetErrorHistory();

        std::vector<Eigen::VectorXd> GetControlInputHistory();

        Eigen::VectorXd GenerateControlInput();

        std::vector<std::string> GetColumnNames();

        friend std::ostream& operator<<(std::ostream& out, const PID& PID);

    private:
        Eigen::MatrixXd kp;
        Eigen::MatrixXd ki;
        Eigen::MatrixXd kd;
        Eigen::VectorXd error;
        Eigen::VectorXd d_error;
        Eigen::VectorXd sum;
        Eigen::VectorXd previous_error;
        std::vector<Eigen::VectorXd> error_history;
        std::vector<Eigen::VectorXd> control_input_history;

};


#endif