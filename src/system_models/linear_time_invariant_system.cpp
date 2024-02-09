# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/linear_time_invariant_system.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

LinearTimeInvariantSys::LinearTimeInvariantSys(VectorXd y0, double t0, 
                                        double dt0, int num_states, 
                                        std::string name,
                                        MatrixXd A_in, MatrixXd B_in) : OdeSolver(y0, t0, dt0), 
                                                                        sys_name(name),
                                                                        A(A_in), B(B_in) {};


VectorXd LinearTimeInvariantSys::f(double t, VectorXd y, VectorXd u) {
    VectorXd yd = A * y + B * u;
    return yd;
};                                                  

std::string LinearTimeInvariantSys::GetName() {
    return sys_name;
}

MatrixXd LinearTimeInvariantSys::GetA() {return A;};

MatrixXd LinearTimeInvariantSys::GetB() {return B;};

std::ostream& operator << (std::ostream& out, const LinearTimeInvariantSys& system) {
    out << "Printing LTI model parameters:\n";
    out << "(A) State transition matrix:\n" << system.A << "\n";
    out << "(B) Input Matrix:\n" << system.B << "\n";
    return out;

};
