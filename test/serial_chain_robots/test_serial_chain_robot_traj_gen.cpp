# include <iostream>
# include <Eigen/Dense>
#include "serial_chain_robots/serial_chain_robot_traj_gen.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void TestCubicTimeScaling() {
    double t_final = 2;
    double t0 = 0;
    double s0 = CubicTimeScaling(t_final, t0);
    double sf = CubicTimeScaling(t_final, t_final);

    std::cout << "\n TestCubicTimeScaling \n";
    std::cout << "\ns(0) = " << s0;
    std::cout << "\ns(t_final) = " << sf;
    std::cout << "\n";

    if (s0 == 0 && sf == 1) {
        std::cout << "test successful!\n";
    } else {
        std::cout << "test failed!\n";
    }

}

void TestQuinticTimeScaling() {
    double t_final = 2;
    double t0 = 0;
    double s0 = QuinticTimeScaling(t_final, t0);
    double sf = QuinticTimeScaling(t_final, t_final);

    std::cout << "\n TestQuinticTimeScaling \n";
    std::cout << "\ns(0) = " << s0;
    std::cout << "\ns(t_final) = " << sf;
    std::cout << "\n";
    
    if (s0 == 0 && sf == 1) {
        std::cout << "test successful!\n";
    } else {
        std::cout << "test failed!\n";
    }

}

void TestJointTrajectory() {
    double q_0 = 0.1;
    double q_f = 1.0;
    double t_final = 2;
    int traj_length = 10;
    std::string time_scaling_type = "cubic";

    std::vector<double> q_traj = JointTrajectory(q_0, q_f, 
                                                t_final, traj_length, 
                                                time_scaling_type);

    std::cout << "\n TestJointTrajectory \n";
    std::cout << "\nq(0) = " << q_traj[0];
    std::cout << "\nq(t_final) = " << q_traj.back();
    std::cout << "\n";
    
    if (q_traj[0] == q_0 && q_traj.back() == q_f) {
        std::cout << "test successful!\n";
    } else {
        std::cout << "test failed!\n";
    }
}

void TestTfScrewTrajectory() {
    MatrixXd TF_0(4,4);
    TF_0 << 1, 0, 0, 1,
            0, 1, 0, 0,
            0, 0, 1, 1,
            0, 0, 0, 1;

    MatrixXd TF_final(4,4);
    TF_final << 0, 0, 1, 0.1,
            1, 0, 0,   0,
            0, 1, 0, 4.1,
            0, 0, 0,   1;

    double t_final = 5;
    int traj_length = 4;
    std::string time_scaling_type = "cubic";

    std::vector<MatrixXd> TF_traj = TfScrewTrajectory(TF_0, TF_final, 
                                                     t_final, traj_length, 
                                                     time_scaling_type);

    std::cout << "\n TestTfScrewTrajectory \n";
    // for (const auto& TF : TF_traj) {
    //     std::cout << TF;
    //     std::cout << "\n";
    // }

    std::cout << "\nTF_0 error= " << TF_traj[0] - TF_0;
    std::cout << "\nTF_final error = " << TF_traj.back() - TF_final;
    std::cout << "\n";

    if (((TF_traj[0] - TF_0).array().abs() < 1e-8).all() && ((TF_traj.back() - TF_final).array().abs() < 1e-8).all()) {
        std::cout << "test successful!\n";
    } else {
        std::cout << "test failed!\n";
    }
}


void TestTfCartesianTrajectory() {
    MatrixXd TF_0(4,4);
    TF_0 << 1, 0, 0, 1,
            0, 1, 0, 0,
            0, 0, 1, 1,
            0, 0, 0, 1;

    MatrixXd TF_final(4,4);
    TF_final << 0, 0, 1, 0.1,
            1, 0, 0,   0,
            0, 1, 0, 4.1,
            0, 0, 0,   1;

    double t_final = 5;
    int traj_length = 4;
    std::string time_scaling_type = "quintic";

    std::vector<MatrixXd> TF_traj = TfCartesianTrajectory(TF_0, TF_final, 
                                                     t_final, traj_length, 
                                                     time_scaling_type);

    std::cout << "\n TestTfCartesianTrajectory \n";
    // for (const auto& TF : TF_traj) {
    //     std::cout << TF;
    //     std::cout << "\n";
    // }

    std::cout << "\nTF_0 error= " << TF_traj[0] - TF_0;
    std::cout << "\nTF_final error = " << TF_traj.back() - TF_final;
    std::cout << "\n";

    if (((TF_traj[0] - TF_0).array().abs() < 1e-8).all() && ((TF_traj.back() - TF_final).array().abs() < 1e-8).all()) {
        std::cout << "test successful!\n";
    } else {
        std::cout << "test failed!\n";
    }
}

int main() {

    TestCubicTimeScaling();

    TestQuinticTimeScaling();

    TestJointTrajectory();

    TestTfScrewTrajectory();

    TestTfCartesianTrajectory();

    return 0;
}