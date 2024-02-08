#ifndef VAN_DER_POL_OSCILLATOR_H
#define VAN_DER_POL_OSCILLATOR_H

# include <Eigen/Dense>
# include "numerical_solvers/rk_ode_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class VanDerPolOscillator : public OdeSolver {
    public:
        VanDerPolOscillator(VectorXd x0, double t0, double dt0, 
                            int num_states, std::string name, double mu0);

        VectorXd f(double t, VectorXd X, VectorXd u) override;

        MatrixXd CalculateFxJacobian(VectorXd X);

        std::string GetName() override;

        std::vector<std::string> GetColumnNames() override;
        
        VectorXd GetB();

        friend std::ostream& operator << (std::ostream& out, const VanDerPolOscillator& system);

    private:
        int num_states;
        MatrixXd A;
        VectorXd B;
        std::string sys_name;
        double mu;

};

#endif