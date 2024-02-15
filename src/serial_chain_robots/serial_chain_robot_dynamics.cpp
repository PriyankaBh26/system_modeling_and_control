# include <iostream>
# include <tuple>
# include <Eigen/Dense>

# include "serial_chain_robots/serial_chain_robot_helper_funs.h"
# include "serial_chain_robots/serial_chain_robot_dynamics.h"

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
    // tau = M(q)d2q + C(q,dq) + Gforce(q) + J.T(q)Fee

    MatrixXd Mi = MatrixXd::Identity(4,4);
    MatrixXd Ai = MatrixXd::Zero(6,num_joints);
    std::vector<MatrixXd> AdTi;
    MatrixXd Vi(6,num_joints+1);
    MatrixXd Vdi(6,num_joints+1);
    Vdi.block(3,0,3,1) = -g_vec;

    VectorXd Fi = Ftip;
    VectorXd tau(num_joints);

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
        
        tau(i) = Fi.transpose() * Ai.col(i);
    }

    return tau;
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

VectorXd SerialChainRobotDynamics::ForwardDynamics(VectorXd q, VectorXd dq, VectorXd Ftip, VectorXd tau) {
    //  d2q = inv(M(q)) * (tau - C(q,dq) - Gforce(q) - J.T(q) * Fee)
    MatrixXd M = SerialChainRobotDynamics::MassMatrix(q);
    VectorXd C = SerialChainRobotDynamics::VelQuadraticForces(q, dq);
    VectorXd Gforce = SerialChainRobotDynamics::GravityForces(q);
    VectorXd JTFee = SerialChainRobotDynamics::EndEffectorForces(q, Ftip);

    VectorXd d2q = M.inverse() * (tau - C - Gforce - JTFee);

    return d2q;
};

VectorXd SerialChainRobotDynamics::EulerStepUpdate(VectorXd x, VectorXd dxdt, double dt) {
    return x + dt * dxdt;
};

std::vector<VectorXd> SerialChainRobotDynamics::UpdateInverseDynamicsTrajectory(std::vector<VectorXd> q_trajectory, 
                                        std::vector<VectorXd> dq_trajectory, 
                                        std::vector<VectorXd> d2q_trajectory,
                                        std::vector<VectorXd> Ftip_trajectory) {
    int trajectory_length = q_trajectory.size();
    std::vector<VectorXd> tau_trajectory;
    for (int i{0}; i<trajectory_length; i++) {
        VectorXd tau = SerialChainRobotDynamics::InverseDynamics(q_trajectory[i], dq_trajectory[i], 
                                                                 d2q_trajectory[i], g, Ftip_trajectory[i]);
        tau_trajectory.push_back(tau);
    }
    return tau_trajectory;
};



std::tuple<std::vector<VectorXd>, std::vector<VectorXd>, std::vector<VectorXd>>  SerialChainRobotDynamics::UpdateForwardDynamicsTrajectory(VectorXd q, 
                                                                VectorXd dq, 
                                                                double dt,
                                                                std::vector<VectorXd> tau_trajectory,
                                                                std::vector<VectorXd> Ftip_trajectory) {
    int trajectory_length = tau_trajectory.size();
    std::vector<VectorXd> q_trajectory;
    std::vector<VectorXd> dq_trajectory;
    std::vector<VectorXd> d2q_trajectory;

    for (int i{0}; i<trajectory_length; i++) {
        VectorXd d2q = SerialChainRobotDynamics::ForwardDynamics(q, dq, Ftip_trajectory[i], tau_trajectory[i]);
        d2q_trajectory.push_back(d2q);

        q = SerialChainRobotDynamics::EulerStepUpdate(q, dq, dt);
        q_trajectory.push_back(q);

        dq = SerialChainRobotDynamics::EulerStepUpdate(dq, d2q, dt);
        dq_trajectory.push_back(dq);
    }
    return std::make_tuple(q_trajectory, dq_trajectory, d2q_trajectory);
};