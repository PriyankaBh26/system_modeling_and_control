#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

# include <iostream>
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ForwardKinematics {
    public:
        ForwardKinematics(int n, 
                          std::vector<std::string> joint_type, 
                          MatrixXd tf_home, 
                          MatrixXd screw_space, 
                          MatrixXd screw_body);

        void TfInSpaceFrame(VectorXd q);

        void TfInBodyFrame(VectorXd q);

        MatrixXd KthJointMatInSpaceFrame(MatrixXd tf_space_k, VectorXd q, int k);

        MatrixXd KthJointMatInBodyFrame(VectorXd q, int k);

        MatrixXd ExponentialMatrix(VectorXd screw_axis, double q_i, std::string joint_type_i);

        MatrixXd SpaceJacobian(VectorXd q);

        MatrixXd BodyJacobian(VectorXd q);

        VectorXd TwistFromJointVelocity(MatrixXd jacobian, VectorXd qd);
    
        VectorXd JointTorqueFromEEForce(MatrixXd jacobian, VectorXd F);

        MatrixXd GetTfSpace();

        MatrixXd GetTfBody();

        friend std::ostream& operator << (std::ostream&  out, ForwardKinematics& robot);

    private:
        int num_joints;
        std::vector<std::string> joint_type;
        MatrixXd tf_home;
        MatrixXd screw_space;
        MatrixXd screw_body;
        MatrixXd tf_space;
        MatrixXd tf_body;
};


#endif