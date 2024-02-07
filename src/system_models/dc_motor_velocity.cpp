# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "system_models/dc_motor_velocity.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

DCMotorVelocity::DCMotorVelocity(VectorXd y0, double t0, double dt0,
            int n, std::string name, double J0,
            double b0, double K0, double R0, double L0) : OdeSolver(y0, t0, dt0),
                                                          num_states(n), A(n, n), B(n), sys_name(name), 
                                                          J(J0), b(b0), K(K0), R(R0), L(L0) {
                                                            A << -b/J, K/J,
                                                                -K/L, -R/L;
                                                            B << 0, 1/L;
                                                          };

VectorXd DCMotorVelocity::f(double t, VectorXd y, VectorXd u) {
    VectorXd yd = A * y + B * u;
    return yd;
};

std::string DCMotorVelocity::GetName() {return sys_name;};

std::vector<std::string> DCMotorVelocity::GetColumnNames() {return {"Vel", "Acc"};};

MatrixXd DCMotorVelocity::GetA() {return A;};

VectorXd DCMotorVelocity::GetB() {return B;};
