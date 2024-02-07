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
                                                                        B(num_states), 
                                                                        sys_name(name),
                                                                        k(k0), c(c0), m(m0) {
                                                                        A << 0, 1,
                                                                            -k/m, -c/m;
                                                                        B << 0, 1;
                                                                        };

MassSpringDamperSys::MassSpringDamperSys(VectorXd y0, double t0, 
                                        double dt0, int num_states, int num_inputs, std::string name,
                                        double k0, double c0, double m0) : OdeSolver(y0, t0, dt0), 
                                                                        A(num_states, num_states), 
                                                                        B(num_states, num_inputs), 
                                                                        sys_name(name),
                                                                        k(k0), c(c0), m(m0) {
                                                                        A << 0, 1,
                                                                            -k/m, -c/m;
                                                                        };


VectorXd MassSpringDamperSys::f(double t, VectorXd y, VectorXd u) {
    VectorXd yd = A * y + B * u;
    return yd;
};                                                  

void MassSpringDamperSys::SetB(MatrixXd B_in) {
    B = B_in;
};


std::string MassSpringDamperSys::GetName() {
    return sys_name;
}

std::vector<std::string> MassSpringDamperSys::GetColumnNames() {
    return {"Pos", "Vel"};
}

MatrixXd MassSpringDamperSys::GetA() {return A;};

VectorXd MassSpringDamperSys::GetB() {return B;};
