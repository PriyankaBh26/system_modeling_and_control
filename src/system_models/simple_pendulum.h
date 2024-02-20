#ifndef SIMPLE_PENDULUM_H
#define SIMPLE_PENDULUM_H

# include <Eigen/Dense>
# include "numerical_solvers/rk_ode_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SimplePendulum: public OdeSolver {
    public:
        SimplePendulum(VectorXd y0, double t0, 
                        double dt0, int num_states, 
                        std::string sys_name, double m, double b, double l);

        VectorXd f(double t, VectorXd y, VectorXd u) override;

        MatrixXd CalculateFxJacobian(VectorXd X);

        std::string GetName() override;

        std::vector<std::string> GetColumnNames() override;
        
        VectorXd GetB();

        friend std::ostream& operator << (std::ostream& out, const SimplePendulum& system);
             

    private:
        int num_states;
        MatrixXd A;
        VectorXd B;
        std::string sys_name;
        double m;
        double b;
        double l;


};

#endif