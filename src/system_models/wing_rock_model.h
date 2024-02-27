#ifndef WING_ROCK_MODEL_H
#define WING_ROCK_MODEL_H


# include <Eigen/Dense>
# include "numerical_solvers/rk_ode_solver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


class WingRockModel : public OdeSolver {
    public:
        WingRockModel(VectorXd y0, double t0, double dt0, 
                        int n, std::string name,
                        MatrixXd A_in, MatrixXd B_in);

        VectorXd f(double t, Eigen::VectorXd X, Eigen::VectorXd u) override;

        std::string GetName() override;

        VectorXd NonlinearDisturbance(Eigen::VectorXd X);

        friend std::ostream& operator << (std::ostream& out, const WingRockModel& system);

        VectorXd GetA();

        VectorXd GetB();

        std::vector<VectorXd> GetDisturbance();


    private:
        int num_states;
        std::string sys_name;
        double dt;
        MatrixXd A;
        MatrixXd B;
        std::vector<VectorXd> disturbance_history;

};

#endif