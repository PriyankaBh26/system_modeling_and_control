# include <vector>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/simple_pendulum.h"

SimplePendulum::SimplePendulum(VectorXd y0, double t0, 
                        double dt0, int n, 
                        std::string name, double m_in, 
                        double b_in, double l_in) : OdeSolver(y0, t0, dt0),
                                                num_states(n),
                                                A(n, n), B(n,1),
                                                sys_name(name), 
                                                m(m_in), b(b_in), l(l_in) {
                                                    B << 0, 1; };

VectorXd SimplePendulum::f(double t, VectorXd X, VectorXd u) {
    VectorXd xd(num_states);
    xd << X(1), -9.81/l * sin(X(0)) - b/(m*l*l) * X(1) + u(0)/(m*l*l);
    return xd;
};

MatrixXd SimplePendulum::CalculateFxJacobian(VectorXd X) {
    A << 0, 1,
         -9.81/l * cos(X(0)), -b/(m*l*l);
    return A;
};

std::string SimplePendulum::GetName() {
    return sys_name;
};

std::vector<std::string> SimplePendulum::GetColumnNames() {
    return {"theta", "omega"};
};

VectorXd SimplePendulum::GetB() {return B;};

std::ostream& operator << (std::ostream& out, const SimplePendulum& system) {
    out << "Printing " << system.sys_name << " model parameter:\n";
    out << "m " << system.m << "\n";
    out << "b " << system.b << "\n";
    out << "l " << system.l << "\n";
    return out;
};