# include <iostream>
# include <Eigen/Dense>

# include "multi_joint_robots/multi_joint_robots_helper_funs.h"
# include "multi_joint_robots/forward_dynamics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ForwardDynamics::ForwardDynamics(int n, 
                                VectorXd q_in,
                                VectorXd dq_in,
                                VectorXd tau_in,
                                VectorXd g_in,
                                VectorXd Ftip_in,
                    std::vector<MatrixXd> Mlist_in,
                    std::vector<MatrixXd> Glist_in,
                                MatrixXd screw_space_in) : 
                                                           num_joints(n),
                                                           q(q_in), 
                                                           dq(dq_in), 
                                                           tau(tau_in), 
                                                           g(g_in), 
                                                           Ftip(Ftip_in), 
                                                           Mlist(Mlist_in),
                                                           Glist(Glist_in), 
                                                           screw_space(screw_space_in) {};
VectorXd ForwardDynamics::InverseDynamics(VectorXd q, 
                                        VectorXd dq, 
                                        VectorXd d2q, 
                                        VectorXd g, 
                                        VectorXd Ftip, 
                                        std::vector<MatrixXd> Mlist, 
                                        std::vector<MatrixXd> Glist, 
                                        MatrixXd screw_space) {
    MatrixXd Mi = MatrixXd::Identity(4,4);
    MatrixXd Ai = MatrixXd::Zero(6,num_joints);
    std::vector<MatrixXd> AdTi;
    MatrixXd Vi(6,num_joints+1);
    MatrixXd Vdi(6,num_joints+1);
    Vdi.block(3,0,3,1) = -g;

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

MatrixXd ForwardDynamics::MassMatrix(VectorXd q,
                                    std::vector<MatrixXd> Mlist, 
                                    std::vector<MatrixXd> Glist, 
                                    MatrixXd screw_space) {

    MatrixXd M(num_joints, num_joints);
    VectorXd dq(num_joints);
    for (int i{0}; i<num_joints; i++) {
        VectorXd d2q(num_joints);
        d2q(i) = 1;
        M.col(i) = ForwardDynamics::InverseDynamics(q, dq, d2q, 
                                                    0*g, 0*Ftip, Mlist, 
                                                    Glist, screw_space);
    }
    return M;
};

VectorXd ForwardDynamics::VelQuadraticForces(VectorXd q,
                                    VectorXd dq,
                                    std::vector<MatrixXd> Mlist, 
                                    std::vector<MatrixXd> Glist, 
                                    MatrixXd screw_space) {

    VectorXd C = ForwardDynamics::InverseDynamics(q, dq, 0*q, \
                           0*g, 0*Ftip, Mlist, Glist, \
                           screw_space);

    return C;
};

VectorXd ForwardDynamics::GravityForces(VectorXd q,
                            std::vector<MatrixXd> Mlist, 
                            std::vector<MatrixXd> Glist, 
                            MatrixXd screw_space) {
    
    VectorXd Gforce = ForwardDynamics::InverseDynamics(q, 0 * q, 0 * q, g,
                                                        0 * Ftip, Mlist, Glist, screw_space);
    return Gforce;
};

VectorXd ForwardDynamics::EndEffectorForces(VectorXd q,
                                            std::vector<MatrixXd> Mlist, 
                                            std::vector<MatrixXd> Glist, 
                                            MatrixXd screw_space) {

    VectorXd Fee = ForwardDynamics::InverseDynamics(q, 0 * q, 0 * q, 0 * g, Ftip,
                                                    Mlist, Glist, screw_space);
    return Fee;
};