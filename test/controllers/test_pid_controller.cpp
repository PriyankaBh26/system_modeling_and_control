# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>
# include "controllers/pid_controller.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void TestSetGains(PID* pid) {
    VectorXd Kp(1);
    Kp << 1.0;

    VectorXd Ki(1);
    Ki << 1.0;

    VectorXd Kd(1);
    Kd << 1.0;

    pid->SetGains(Kp, Ki, Kd);

    std::cout << "\n TestGenerateControlInput \n";
    std::cout << *pid;
    std::cout << "\n";

};

void TestGenerateControlInput(PID* pid) {
    VectorXd x_err(1);
    x_err << 1.0;

    VectorXd dxdt_err(1);
    dxdt_err << 1.0;

    VectorXd u_expected(1);
    u_expected << 3.0;

    VectorXd u = pid->GenerateControlInput(x_err, dxdt_err);
    std::cout << "\n TestGenerateControlInput \n";
    std::cout << "\n control ip: " << u;
    
    if (((u - u_expected).array().abs() < 1e-5).all()) {
        std::cout << "\ntest successful!\n";
    } else {
        std::cout << "\ntest failed!\n";
    }
    std::cout << "\n";

};


void TestPIDController() {
    int num_x = 1;
    PID* pid = new PID(num_x);

    TestSetGains(pid);

    TestGenerateControlInput(pid);

    delete pid;
};

int main() {

    TestPIDController();

    return 0;
};