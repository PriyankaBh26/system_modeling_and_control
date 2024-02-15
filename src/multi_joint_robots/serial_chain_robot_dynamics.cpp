# include <iostream>
# include <Eigen/Dense>

# include "multi_joint_robots/serial_chain_robot_helper_funs.h"
# include "multi_joint_robots/serial_chain_robot_dynamics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

SerialChainRobotDynamics::SerialChainRobotDynamics(int n, 
                                VectorXd g_in,
                    std::vector<MatrixXd> Mlist_in,
                    std::vector<MatrixXd> Glist_in,
                                MatrixXd screw_space_in) : 
                                                           num_joints(n),
                                                           g(g_in), 
                                                           Mlist(Mlist_in),
                                                           Glist(Glist_in), 
                                                           screw_space(screw_space_in) {};
VectorXd SerialChainRobotDynamics::InverseDynamics(VectorXd q, 
                                        VectorXd dq, 
                                        VectorXd d2q, 
                                        VectorXd g_vec,
                                        VectorXd Ftip) {
    MatrixXd Mi = MatrixXd::Identity(4,4);
    MatrixXd Ai = MatrixXd::Zero(6,num_joints);
    std::vector<MatrixXd> AdTi;
    MatrixXd Vi(6,num_joints+1);
    MatrixXd Vdi(6,num_joints+1);
    Vdi.block(3,0,3,1) = -g_vec;

    VectorXd Fi = Ftip;
    VectorXd tau_list(num_joints);

    for (int i{0}; i<num_joints; i++) {
        Mi = Mi * Mlist[i];
        
        Ai.col(i) = AdjointOfTfMatrix(TfmatInverse(Mi)) * screw_space.col(i);
        
        AdTi.push_back(AdjointOfTfMatrix(MatrixExp6(VecToSe3(Ai.col(i) * (-q(i)))) * TfmatInverse(Mlist[i])));
        
        Vi.col(i+1) = AdTi.back() * Vi.col(i) + Ai.col(i) * dq(i);

        Vdi.col(i+1) = AdTi.back() * Vdi.col(i) + Ai.col(i) * d2q(i) + ad(Vi.col(i+1)) * Ai.col(i) * dq(i);
    }

    AdTi.push_back(AdjointOfTfMatrix(TfmatInverse(Mlist[num_joints])));

    for (int i{num_joints-1}; i>=0; i--) {
        Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1) - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
        
        tau_list(i) = Fi.transpose() * Ai.col(i);
    }

    return tau_list;
};

MatrixXd SerialChainRobotDynamics::MassMatrix(VectorXd q) {

    MatrixXd M(num_joints, num_joints);
    VectorXd dq(num_joints);
    VectorXd zero_F(6);
    for (int i{0}; i<num_joints; i++) {
        VectorXd d2q(num_joints);
        d2q(i) = 1;
        M.col(i) = SerialChainRobotDynamics::InverseDynamics(q, dq, d2q, 0*g, zero_F);
    }
    return M;
};

VectorXd SerialChainRobotDynamics::VelQuadraticForces(VectorXd q, VectorXd dq) {
    VectorXd zero_F(6);
    VectorXd C = SerialChainRobotDynamics::InverseDynamics(q, dq, 0*q, 0*g, zero_F);

    return C;
};

VectorXd SerialChainRobotDynamics::GravityForces(VectorXd q) {
    VectorXd zero_F(6);
    VectorXd Gforce = SerialChainRobotDynamics::InverseDynamics(q, 0 * q, 0 * q, g, zero_F);
    return Gforce;
};

VectorXd SerialChainRobotDynamics::EndEffectorForces(VectorXd q, VectorXd Ftip) {

    VectorXd Fee = SerialChainRobotDynamics::InverseDynamics(q, 0 * q, 0 * q, 0 * g, Ftip);
    return Fee;
};