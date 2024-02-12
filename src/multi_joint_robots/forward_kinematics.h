#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ForwardKinematics {
    public:
        ForwardKinematics(int n, std::vector<std::string> joint_type, 
                          MatrixXd tf_home, MatrixXd screw_space, MatrixXd screw_body);

        void TfInSpaceFrame(VectorXd q);

        void TfInBodyFrame(VectorXd q);

        MatrixXd KthJointMatInSpaceFrame(VectorXd q, int k);

        MatrixXd KthJointMatInBodyFrame(VectorXd q, int k);

        MatrixXd ExponentialMatrix(VectorXd screw_axis, double q_i, std::string joint_type_i);

        MatrixXd RotMatPosToTFMat(MatrixXd R, VectorXd p);

        MatrixXd SpaceJacobian(VectorXd q);

        MatrixXd BodyJacobian(VectorXd q);

        MatrixXd CalculateAdjointOfTfMatrix(MatrixXd tf_mat);

        VectorXd CalculateTwist(MatrixXd jacobian, VectorXd qd);
    
        VectorXd CalculateJointTorques(MatrixXd jacobian, VectorXd F);

        MatrixXd VecToSkewSymmetricMat(VectorXd v);

        MatrixXd GetTfSpace();

        MatrixXd GetTfBody();

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