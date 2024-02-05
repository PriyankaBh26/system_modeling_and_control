# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class MassSpringDamper : public OdeSolver {
    public: 

        MassSpringDamper(VectorXd x0, double t0, double dt0) : OdeSolver(x0, t0, dt0) {};

        VectorXd f(double t, VectorXd x, VectorXd u) override {
            double k = 1.0;
            double c = 0.01;
            double m = 1.0;

            VectorXd xd(2);
            xd << x[1] + u[0], - k/m * x[0] - c/m * x[1] + u[1];
            return xd;
        }

        std::string GetName() override {
            return "mass_spring_damper";
        }

        std::vector<std::string> GetColumnNames() override {
            return {"Pos", "Vel"};
        }
};

class VanDerPolOscillator : public OdeSolver {
    public: 

        VanDerPolOscillator(VectorXd x0, double t0, double dt0) : OdeSolver(x0, t0, dt0) {};

        VectorXd f(double t, VectorXd x, VectorXd u) override {
            double mu = 2.5;
            
            VectorXd xd(2);
            xd << x[1] + u[0], mu * (1 - std::pow(x[0], 2)) * x[1] - x[0] + u[1];

            return xd;
        }

        std::string GetName() override {
            return "van_der_pol";
        }

        std::vector<std::string> GetColumnNames() override {
            return {"Pos", "Vel"};
        }
};


int main () {
    
    int num_states = 2; 
    VectorXd x0(num_states);
    x0 << 1.0, 0;
    double t0 = 0.0;
    double dh = 0.0001;

    std::unique_ptr<OdeSolver> ode (new VanDerPolOscillator(x0, t0, dh));

    // set integration duration
    double time_final = 1;
    int ode_timesteps = time_final/dh;

    // initialize control input
    VectorXd u(num_states);

    ode->IntegrateODE(ode_timesteps, u);

    std::vector<VectorXd> x_history = ode->GetXHistory();
    std::vector<double> t_history = ode->GetTHistory();

    std::cout << x_history.size() << " " << x_history[0].size();

    std::string filename = "examples/" + ode->GetName() + "_solution.csv";
    std::vector<std::string> columnNames = {"Pos", "Vel"};

    WriteMatToFile(filename, columnNames, x_history);
    
    filename = "examples/" + ode->GetName() + "_time.csv";
    columnNames = {"time"};
    WriteVecToFile(filename, columnNames, t_history);

    return 0;
}