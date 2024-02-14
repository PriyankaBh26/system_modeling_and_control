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
                          double tolerance,
                          int max_iterations);


        MatrixXd TfBody(VectorXd q);

        MatrixXd TfSpace(VectorXd q);

        VectorXd f(VectorXd q) override;

        MatrixXd dfdq(VectorXd q) override;
        
        void SetTfDesired(MatrixXd tf_des);

        void SetInitialJointAngles(VectorXd q0);

        VectorXd SolveIK(MatrixXd tf_des, 
                        VectorXd q0);

        friend std::ostream& operator << (std::ostream& out, InverseKinematics& system);
    
    private:
        std::vector<std::string> joint_type;
        MatrixXd tf_home;
        MatrixXd screw_space;
        MatrixXd screw_body;
        std::string desired_config_type;
        MatrixXd tf_desired;
        VectorXd q;
        double tolerance;
        int max_iterations;

};


#endif