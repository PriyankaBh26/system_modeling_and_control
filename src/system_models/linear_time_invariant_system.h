#ifndef LINEAR_TIME_INVARIANT_SYSTEM_H
#define LINEAR_TIME_INVARIANT_SYSTEM_H

# include <Eigen/Dense>
# include "numerical_solvers/rk_ode_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class LinearTimeInvariantSys : public OdeSolver {
    public: 
        LinearTimeInvariantSys(VectorXd y0, double t0, 
                                        double dt0, int num_states, std::string name,
                                        MatrixXd A_in, MatrixXd B_in);
        VectorXd f(double t, VectorXd y, VectorXd u) override;

        std::string GetName() override;

        MatrixXd GetA();
        MatrixXd GetB();

        friend std::ostream& operator << (std::ostream& out, const LinearTimeInvariantSys& system);
                                        
    private:
        std::string sys_name;
        MatrixXd A;
        MatrixXd B;
};


#endif