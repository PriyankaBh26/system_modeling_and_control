#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class ForwardKinematics {
    public:
        ForwardKinematics(int n, VectorXd L, std::vector<std::string> joint_type, 
                          MatrixXd tf_0_se, MatrixXd screw_axes_s, MatrixXd screw_axes_e);

        poe_tf_in_base_frame();

        poe_tf_in_ee_frame();

        MatrixXd exponential_matrix(VectorXd screw_axis, double q_i, std::string joint_type_i);

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