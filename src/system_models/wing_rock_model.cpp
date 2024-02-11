# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "system_models/wing_rock_model.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

WingRockModel::WingRockModel(VectorXd y0, double t0, double dt0, 
                                int n, std::string name,
                                MatrixXd A_in, MatrixXd B_in) : OdeSolver(y0, t0, dt0), 
                                                                num_states(n), sys_name(name),
                                                                dt(dt0), A(A_in), B(B_in){};

VectorXd WingRockModel::f(double t, VectorXd X, VectorXd u) {

    VectorXd Xdot = A * X + B * u + WingRockModel::NonlinearDisturbance(X);

    X = X + Xdot * dt;
    return X;
};

VectorXd WingRockModel::NonlinearDisturbance(Eigen::VectorXd X) {
    VectorXd disturbance(num_states); 

    VectorXd phi(6);
    phi << 1, X(0), X(1), abs(X(0)), X(1)*abs(X(1)), std::pow(X(0),3);
    VectorXd w(6);
    w << 0.8, 0.2314, 0.6918, -0.6245, 0.0095, 0.0214;

    disturbance << 0, (w.array() * phi.array()).sum();
    // disturbance << 0, 0;

    disturbance_history.push_back(disturbance);
    return disturbance;
};

std::string WingRockModel::GetName() {return sys_name;};

VectorXd WingRockModel::GetA() {return A;};

VectorXd WingRockModel::GetB() {return B;};

std::vector<VectorXd> WingRockModel::GetDisturbance() {return disturbance_history;};

std::ostream& operator << (std::ostream& out, const WingRockModel& system) {
    out << "\nWing rock model parameters:\n";
    out << "A:\n" << system.A;
    out << "\nB:\n" << system.B;
    return out;
}