# include <Eigen/Dense>

# include "multi_joint_robots/forward_dynamics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

ForwardDynamics::ForwardDynamics(VectorXd q_in,
                                VectorXd dq_in,
                                VectorXd tau_in,
                                VectorXd g_in,
                                VectorXd Ftip_in,
                    std::vector<MatrixXd> Mlist_in,
                    std::vector<MatrixXd> Glist_in,
                                MatrixXd screw_space_in) : q(q_in), 
                                                           dq(dq_in), 
                                                           tau(tau_in), 
                                                           g(g_in), 
                                                           Ftip(Ftip_in), 
                                                           Mlist(Mlist_in),
                                                           Glist(Glist_in), 
                                                           screw_space(screw_space_in) {};

MatrixXd ForwardDynamics::MassMatrix();

VectorXd ForwardDynamics::VelQuadraticForces();

VectorXd ForwardDynamics::GravityForces();

VectorXd ForwardDynamics::EndEffectorForces();