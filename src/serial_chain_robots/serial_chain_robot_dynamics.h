#ifndef SERIAL_CHAIN_ROBOT_DYNAMICS_H
#define SERIAL_CHAIN_ROBOT_DYNAMICS_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SerialChainRobotDynamics {
    public:
        SerialChainRobotDynamics(int num_joints,
                        VectorXd g, 
            std::vector<MatrixXd> Mlist,
            std::vector<MatrixXd> Glist, 
                        MatrixXd screw_space);

        VectorXd InverseDynamics(VectorXd q, VectorXd dq, VectorXd d2q, VectorXd g_vec, VectorXd Ftip);

        MatrixXd MassMatrix(VectorXd q);

        VectorXd VelQuadraticForces(VectorXd q, VectorXd dq);

        VectorXd GravityForces(VectorXd q);

        VectorXd EndEffectorForces(VectorXd q, VectorXd Ftip);

        VectorXd ForwardDynamics(VectorXd q, VectorXd dq, VectorXd Ftip, VectorXd tau);

        VectorXd EulerStepUpdate(VectorXd x, VectorXd dxdt, double dt);

        std::vector<VectorXd> UpdateInverseDynamicsTrajectory(std::vector<VectorXd> q_trajectory, 
                                                            std::vector<VectorXd> dq_trajectory, 
                                                            std::vector<VectorXd> d2q_trajectory,
                                                            std::vector<VectorXd> Ftip_trajectory);
    private:
        int num_joints;
        VectorXd g; 
        std::vector<MatrixXd> Mlist;
        std::vector<MatrixXd> Glist; 
        MatrixXd screw_space;

};


#endif