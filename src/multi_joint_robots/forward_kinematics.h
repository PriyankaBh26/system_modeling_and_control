#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ForwardKinematics {
    public:
        ForwardKinematics(int n, VectorXd L, std::vector<std::string> joint_type, 
                          MatrixXd tf_0_se, MatrixXd screw_axes_s, MatrixXd screw_axes_e);

        void TfMatrixInBaseFrame();

        void TfMatrixInEEFrame();

        MatrixXd KthJointMatrixInBaseFrame(int k);

        MatrixXd KthJointMatrixInEEFrame(int k);

        MatrixXd ExponentialMatrix(VectorXd screw_axis, double q_i, std::string joint_type_i);

        MatrixXd SpaceJacobian();

        MatrixXd BodyJacobian();

        MatrixXd CalculateAdjointOfTfMatrix(MatrixXd tf_mat);

        VectorXd CalculateEETwist(MatrixXd jacobian, VectorXd qd);
    
        VectorXd CalculateJointTorques(MatrixXd jacobian, VectorXd F);

    private:
        int num_joints;
        VectorXd link_length;
        std::vector<std::string> joint_type;
        MatrixXd tf_0_se;
        MatrixXd screw_axes_s;
        MatrixXd screw_axes_e;
        VectorXd q;
        MatrixXd tf_se;
        MatrixXd tf_es;
};


#endif