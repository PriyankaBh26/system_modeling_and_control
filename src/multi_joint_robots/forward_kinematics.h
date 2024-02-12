#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ForwardKinematics {
    public:
        ForwardKinematics(int n, VectorXd L, std::vector<std::string> joint_type, 
                          MatrixXd tf_home, MatrixXd screw_axes_s, MatrixXd screw_axes_e);

        void TfMatrixInBaseFrame(VectorXd q);

        void TfMatrixInEEFrame(VectorXd q);

        MatrixXd KthJointMatrixInBaseFrame(VectorXd q, int k);

        MatrixXd KthJointMatrixInEEFrame(VectorXd q, int k);

        MatrixXd ExponentialMatrix(VectorXd screw_axis, double q_i, std::string joint_type_i);

        MatrixXd SpaceJacobian(VectorXd q);

        MatrixXd BodyJacobian(VectorXd q);

        MatrixXd CalculateAdjointOfTfMatrix(MatrixXd tf_mat);

        VectorXd CalculateEETwist(MatrixXd jacobian, VectorXd qd);
    
        VectorXd CalculateJointTorques(MatrixXd jacobian, VectorXd F);

        MatrixXd VecToSkewSymmetricMat(VectorXd v);

        MatrixXd GetTfse();

        MatrixXd GetTfes();

    private:
        int num_joints;
        VectorXd link_length;
        std::vector<std::string> joint_type;
        MatrixXd tf_home;
        MatrixXd screw_axes_s;
        MatrixXd screw_axes_e;
        MatrixXd tf_se;
        MatrixXd tf_es;
};


#endif