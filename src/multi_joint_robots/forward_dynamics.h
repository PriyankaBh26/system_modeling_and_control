#ifndef FORWARD_DYNAMICS_H
#define FORWARD_DYNAMICS_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// This function computes d2q by solving:
//    d2q = inv(Mlist(q)) * (tau - c(q,dq) - g(q) - Jtr(q) * Ftip)

class ForwardDynamics {
    public:
        ForwardDynamics(int num_joints,
                        VectorXd q, 
                        VectorXd dq, 
                        VectorXd taulist, 
                        VectorXd g, 
                        VectorXd Ftip, 
            std::vector<MatrixXd> Mlist,
            std::vector<MatrixXd> Glist, 
                        MatrixXd screw_space);

        VectorXd InverseDynamics(VectorXd q, 
                                 VectorXd dq, 
                                 VectorXd d2q, 
                                 VectorXd g, 
                                 VectorXd Ftip, 
                                 std::vector<MatrixXd> Mlist, 
                                 std::vector<MatrixXd> Glist, 
                                 MatrixXd screw_space);

        MatrixXd MassMatrix(VectorXd q,
                            std::vector<MatrixXd> Mlist, 
                            std::vector<MatrixXd> Glist, 
                            MatrixXd screw_space);

        VectorXd VelQuadraticForces(VectorXd q,
                                    VectorXd dq,
                                    std::vector<MatrixXd> Mlist, 
                                    std::vector<MatrixXd> Glist, 
                                    MatrixXd screw_space);

        // VectorXd GravityForces();

        // VectorXd EndEffectorForces();

    private:
        int num_joints;
        VectorXd q; 
        VectorXd dq; 
        VectorXd tau; 
        VectorXd g; 
        VectorXd Ftip; 
        std::vector<MatrixXd> Mlist;
        std::vector<MatrixXd> Glist; 
        MatrixXd screw_space;

};


#endif