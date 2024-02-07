#ifndef MASS_SPRING_DAMPER_H
#define MASS_SPRING_DAMPER_H

# include <Eigen/Dense>
# include "numerical_solvers/rk_ode_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MassSpringDamperSys : public OdeSolver {
    public: 
    MassSpringDamperSys(VectorXd y0, double t0, double dt0, 
                        int num_states, std::string name, 
                        double k0, double c0, double m0);

    MassSpringDamperSys(VectorXd y0, double t0, double dt0, 
                        int num_states, int num_inputs, std::string name, 
                        double k0, double c0, double m0);

    VectorXd f(double t, VectorXd y, VectorXd u) override;

    void SetB(MatrixXd B_in);

    std::string GetName() override;

    std::vector<std::string> GetColumnNames() override;

    MatrixXd GetA();
    VectorXd GetB();

    private:
        MatrixXd A;
        VectorXd B;
        std::string sys_name;
        double k;
        double c;
        double m;

};

#endif