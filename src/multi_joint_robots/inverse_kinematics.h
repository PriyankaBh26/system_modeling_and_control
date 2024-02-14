#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

# include <iostream>
# include <Eigen/Dense>
# include "numerical_solvers/newton_raphson.h"
# include "multi_joint_robots/forward_kinematics.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class InverseKinematics: public ForwardKinematics, public NewtonRaphson {
    public:
        InverseKinematics(int num_joints, 
                          std::vector<std::string> joint_type, 
                          MatrixXd tf_home, 
                          MatrixXd screw_space, 
                          MatrixXd screw_body,
                          std::string desired_config_type,
                          MatrixXd tf_desired,
                          VectorXd q0,
                          double tolerance,
                          int max_iterations);

        MatrixXd TfmatInverse(MatrixXd Mat);

        MatrixXd TfBody(VectorXd q);

        MatrixXd TfSpace(VectorXd q);

        VectorXd SkewSymMatToVec(MatrixXd W);

        MatrixXd VecToSkewSymMat(VectorXd w);

        MatrixXd MatrixLog3(MatrixXd R);

        VectorXd Se3ToVec(MatrixXd V_B);

        MatrixXd MatrixLog6(MatrixXd tf_mat);

        VectorXd f(VectorXd q) override;

        MatrixXd dfdq(VectorXd q) override;
        
        VectorXd SolveIK();

        friend std::ostream& operator << (std::ostream& out, InverseKinematics& system);
    
    private:
        std::vector<std::string> joint_type;
        MatrixXd tf_home;
        MatrixXd screw_space;
        MatrixXd screw_body;
        std::string desired_config_type;
        MatrixXd tf_desired;
        VectorXd q;

};


#endif