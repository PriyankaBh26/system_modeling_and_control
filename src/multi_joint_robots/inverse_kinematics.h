#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class InverseKinematics {
    public:
        InverseKinematics(int n, VectorXd L, std::vector<std::string> joint_type, 
                          MatrixXd tf_0_se, MatrixXd screw_axes_s, MatrixXd screw_axes_e);

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