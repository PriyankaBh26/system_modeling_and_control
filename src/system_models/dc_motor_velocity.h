#ifndef DC_MOTOR_VELOCITY_H
#define DC_MOTOR_VELOCITY_H

# include <Eigen/Dense>
# include "numerical_solvers/rk_ode_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class DCMotorVelocity : public OdeSolver {
    public: 
        DCMotorVelocity(VectorXd y0, double t0, double dt0,
                    int num_states, std::string name, double J,
                    double b, double K, double R, double L);

        VectorXd f(double t, VectorXd y, VectorXd u) override;

        std::string GetName() override;

        std::vector<std::string> GetColumnNames() override;

        MatrixXd GetA();

        VectorXd GetB();

        friend std::ostream& operator<<(std::ostream& out, const DCMotorVelocity& system);

    private:
        int num_states;
        MatrixXd A;
        VectorXd B;
        std::string sys_name;
        double J;
        double b;
        double K;
        double R;
        double L;
};


#endif