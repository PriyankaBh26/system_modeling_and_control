#ifndef SERIAL_CHAIN_ROBOT_PID_CONTROL_H
#define SERIAL_CHAIN_ROBOT_PID_CONTROL_H


# include <iostream>
# include <tuple>
# include <Eigen/Dense>

# include "serial_chain_robots/serial_chain_robot_helper_funs.h"
# include "serial_chain_robots/serial_chain_robot_dynamics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SerialChainRobotPIDControl: public SerialChainRobotDynamics {
    public:
        SerialChainRobotPIDControl(int num_joints,
                                    VectorXd g, 
                                    std::vector<MatrixXd> Mlist,
                                    std::vector<MatrixXd> Glist, 
                                    MatrixXd screw_space);

        void SetPIDGains(VectorXd Kp_in, VectorXd Ki_in, VectorXd Kd_in);

        VectorXd ControlInput(VectorXd q_ref, VectorXd dq_ref, VectorXd q, VectorXd dq);

        VectorXd ComputeTorque(VectorXd q_ref, VectorXd dq_ref, VectorXd d2q_ref, VectorXd q, VectorXd dq);

        std::tuple<VectorXd, VectorXd, VectorXd, VectorXd> ComputedTorqueControl(VectorXd q_ref, VectorXd dq_ref, VectorXd d2q_ref, 
                                                                                VectorXd q, VectorXd dq, 
                                                                                VectorXd q_meas, VectorXd dq_meas,
                                                                                VectorXd Ftip, 
                                                                                double dt, double dh);

        std::vector<VectorXd> GetQErrorHistory();

        std::vector<VectorXd> GetSumErrorHistory();

        std::vector<VectorXd> GetDqErrorHistory();

        std::vector<VectorXd> GetControlHistory();

        std::vector<VectorXd> GetQHistory();

        std::vector<VectorXd> GetDqHistory();

        std::vector<VectorXd> GetD2qHistory();

        std::vector<VectorXd> GetTauHistory();

        friend std::ostream& operator << (std::ostream& out, SerialChainRobotPIDControl& robot);


    private:
        int num_joints;
        VectorXd Kp;
        VectorXd Ki;
        VectorXd Kd;
        VectorXd g;
        std::vector<VectorXd> q_error_history;
        std::vector<VectorXd> sum_error_history;
        std::vector<VectorXd> dq_error_history;
        std::vector<VectorXd> control_history;
        std::vector<VectorXd> q_history;
        std::vector<VectorXd> dq_history;
        std::vector<VectorXd> d2q_history;
        std::vector<VectorXd> tau_history;



};


#endif