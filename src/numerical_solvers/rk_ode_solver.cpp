# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>
# include "numerical_solvers/rk_ode_solver.h"
 
using Eigen::MatrixXd;
using Eigen::VectorXd;

OdeSolver::OdeSolver(VectorXd x0, double t0, double dt0) : x(x0), t(t0), dt(dt0) {
    x_history.push_back(x0);
    t_history.push_back(t0);
};

VectorXd OdeSolver::f(double t, VectorXd X, VectorXd u) {return X;};

void OdeSolver::RK4Update(VectorXd u) {
    // k1 = f(t, x)
    VectorXd k1 = f(t, x, u);
    x = x + dt * k1 / 2;
    
    // k2  = f(t + dt/2, x + dt * k1 / 2)
    VectorXd k2 = f(t + dt/2, x, u);
    x = x + dt * k2 / 2;

    // k3 = f(t + dt/2, x + dt * k2 / 2)
    VectorXd k3 = f(t + dt/2, x, u);
    x = x + dt * k3;

    // k4 = f(t + dt, x + dt * k3)
    VectorXd k4 = f(t + dt, x, u);
    
    // update x
    x = x + dt / 6 * (k1 + 2 * k2  + 2 * k3 + k4);

    // update t
    t = t + dt;

};

void OdeSolver::IntegrateODE(int timesteps, VectorXd u) {
    for (int i{1}; i<=timesteps; i++) {
        this->RK4Update(u);  

        x_history.push_back(x);
        t_history.push_back(t);
    }
    return;
};

std::string OdeSolver::GetName() {return "generic_ode";};

std::vector<std::string> OdeSolver::GetColumnNames() {
    std::vector<std::string> column_names;
    int ip_size = x.size();
    for (int i{0}; i<ip_size; i++) {
        column_names.push_back("x" + std::to_string(i));
    }    
    return column_names;
};

std::vector<std::string> OdeSolver::GetControlInputColumnNames(VectorXd u) {
    std::vector<std::string> column_names;
    int ip_size = u.size();
    for (int i{0}; i<ip_size; i++) {
        column_names.push_back("U" + std::to_string(i));
    }    
    return column_names;
};

VectorXd OdeSolver::GetX() {
    return x;
};

double OdeSolver::GetT() {
    return t;
};

std::vector<VectorXd> OdeSolver::GetXHistory() {return x_history;};

std::vector<double> OdeSolver::GetTHistory() {return t_history;};

OdeSolver::~OdeSolver() {};

