#ifndef RKODESOLVER_H
#define RKODESOLVER_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class OdeSolver {
    public:
        OdeSolver(VectorXd x0, double t0, double dt0);
        virtual ~OdeSolver();

        virtual VectorXd f(double t, VectorXd X, VectorXd u);
        virtual MatrixXd CalculateFxJacobian(VectorXd X);

        virtual VectorXd h(double t, VectorXd X, VectorXd u);
        virtual MatrixXd CalculateHxJacobian(VectorXd X);

        virtual std::string GetName();
        virtual std::vector<std::string> GetColumnNames();
        
        std::vector<std::string> GetControlInputColumnNames(VectorXd u);

        void RK4Update(VectorXd u);
        void IntegrateODE(int timesteps, VectorXd u);

        VectorXd GetX();
        double GetT();
        std::vector<VectorXd> GetXHistory();
        std::vector<double> GetTHistory();

    private:
        VectorXd x;
        double t;
        double dt;
        std::vector<VectorXd> x_history;
        std::vector<double> t_history;

};

#endif
