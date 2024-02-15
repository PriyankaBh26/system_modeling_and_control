#ifndef SERIAL_CHAIN_ROBOT_DYNAMICS_H
#define SERIAL_CHAIN_ROBOT_DYNAMICS_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// This function computes d2q by solving:
//    d2q = inv(Mlist(q)) * (tau - c(q,dq) - g(q) - J.T(q) * Ftip)

class SerialChainRobotDynamics {
    public:
        SerialChainRobotDynamics(int num_joints,
                        VectorXd g, 
            std::vector<MatrixXd> Mlist,
            std::vector<MatrixXd> Glist, 
                        MatrixXd screw_space);

        VectorXd InverseDynamics(VectorXd q, 
                                 VectorXd dq, 
                                 VectorXd d2q, 
                                 VectorXd g_vec,
                                 VectorXd Ftip);

        MatrixXd MassMatrix(VectorXd q);

        VectorXd VelQuadraticForces(VectorXd q, VectorXd dq);

        VectorXd GravityForces(VectorXd q);

        VectorXd EndEffectorForces(VectorXd q, VectorXd Ftip);

    private:
        int num_joints;
        VectorXd g; 
        std::vector<MatrixXd> Mlist;
        std::vector<MatrixXd> Glist; 
        MatrixXd screw_space;

};


#endif