# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/mass_spring_damper.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

MassSpringDamperSys::MassSpringDamperSys(VectorXd y0, double t0, 
                                        double dt0, int num_states, std::string name,
                                        double k0, double c0, double m0) : OdeSolver(y0, t0, dt0), 
                                                                        A(num_states, num_states), 
                                                                        B(num_states, 1), 
                                                                        sys_name(name),
                                                                        k(k0), c(c0), m(m0) {
                                                                        A << 0, 1,
                                                                            -k/m, -c/m;
                                                                        B << 0, 1;
                                                                        };

MassSpringDamperSys::MassSpringDamperSys(VectorXd y0, double t0, 
                                        double dt0, int num_states, MatrixXd B_in, std::string name,
                                        double k0, double c0, double m0) : OdeSolver(y0, t0, dt0), 
                                                                        A(num_states, num_states), 
                                                                        B(B_in), 
                                                                        sys_name(name),
                                                                        k(k0), c(c0), m(m0) {
                                                                        A << 0, 1,
                                                                            -k/m, -c/m;
                                                                        };


VectorXd MassSpringDamperSys::f(double t, VectorXd y, VectorXd u) {
    VectorXd yd = A * y + B * u;
    return yd;
};                                                  

std::string MassSpringDamperSys::GetName() {
    return sys_name;
}

std::vector<std::string> MassSpringDamperSys::GetColumnNames() {
    return {"Pos", "Vel"};
}

MatrixXd MassSpringDamperSys::GetA() {return A;};

MatrixXd MassSpringDamperSys::GetB() {return B;};

std::ostream& operator << (std::ostream& out, const MassSpringDamperSys& system) {
    out << "Printing mass-spring-damper model parameters:\n";
    out << "(k) Spring constant  " << system.k << "N/m\n";
    out << "(c) Damping constant " << system.c << "N/m.s\n";
    out << "(m) Mass             " << system.m << "Kg\n";
    return out;

};
