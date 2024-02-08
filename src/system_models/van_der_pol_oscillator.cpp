# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/van_der_pol_oscillator.h"

VanDerPolOscillator::VanDerPolOscillator(VectorXd x0, double t0, double dt0, 
                    int n, std::string name, double mu0) : OdeSolver(x0, t0, dt0), 
                                                    num_states(n),
                                                    A(n, n), B(n), 
                                                    sys_name(name), mu(mu0) {
                                                    B << 0, 1; };

VectorXd VanDerPolOscillator::f(double t, VectorXd X, VectorXd u) {
    VectorXd xd(num_states);
    xd << X[1], mu * (1 - std::pow(X[0], 2)) * X[1] - X[0];
    xd += B * u;
    return xd;
};

MatrixXd VanDerPolOscillator::CalculateFxJacobian(VectorXd X) {
    A << 0, 1, 
        mu * 2 * X[0] * X[1] - 1, mu *  (1 - std::pow(X[0], 2));
    return A;
};

std::string VanDerPolOscillator::GetName() {
    return sys_name;
};

std::vector<std::string> VanDerPolOscillator::GetColumnNames() {
    return {"Pos", "Vel"};
};

VectorXd VanDerPolOscillator::GetB() {return B;};

std::ostream& operator << (std::ostream& out, const VanDerPolOscillator& system) {
    out << "Printing Van der Pol oscillator model parameter:\n";
    out << "mu " << system.mu << "\n";
    return out;
};
