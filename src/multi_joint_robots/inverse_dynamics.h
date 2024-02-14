#ifndef INVERSE_DYNAMICS_H
#define INVERSE_DYNAMICS_H

# include <iostream>
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class InverseDynamics {
    public:
        InverseDynamics(int num_joints,
                        VectorXd thetalist,
                        VectorXd dthetalist,
                        VectorXd ddthetalist,
                        VectorXd g,
                        VectorXd Ftip,
                        std::Vector<MatrixXd> Mlist,
                        std::Vector<VectorXd> Glist,
                        MatrixXd Slist);

        MatrixXd ad

    private:
        int num_joints;
        VectorXd thetalist; //n-vector of joint variables
        VectorXd dthetalist; //n-vector of joint rates
        VectorXd ddthetalist; //n-vector of joint accelerations
        VectorXd g; // Gravity vector g
        VectorXd Ftip; //Spatial force applied by the end-effector expressed in frame {n+1}
        std::Vector<MatrixXd> Mlist; //List of link frames {i} relative to {i-1} at the home position
        std::Vector<VectorXd> Glist; //Spatial inertia matrices Gi of the links
        MatrixXd Slist //Screw axes Si of the joints in a space frame, in the format
                  // of a matrix with axes as the columns
}

#endif