#ifndef RKODESOLVER_H
#define RKODESOLVER_H

# include <Eigen/Dense>

class OdeSolver {
    public:
        OdeSolver(Eigen::VectorXd x0, double t0, double dt0);
        virtual ~OdeSolver();

        virtual Eigen::VectorXd f(double t, Eigen::VectorXd X, Eigen::VectorXd u);
        virtual MatrixXd CalculateFxJacobian(VectorXd X);

        virtual Eigen::VectorXd h(double t, Eigen::VectorXd X, Eigen::VectorXd u);
        virtual MatrixXd CalculateHxJacobian(VectorXd X);

        virtual std::string GetName();
        virtual std::vector<std::string> GetColumnNames();
        
        std::vector<std::string> GetControlInputColumnNames(Eigen::VectorXd u);

        void RK4Update(Eigen::VectorXd u);
        void IntegrateODE(int timesteps, Eigen::VectorXd u);

        Eigen::VectorXd GetX();
        double GetT();
        std::vector<Eigen::VectorXd> GetXHistory();
        std::vector<double> GetTHistory();

    private:
        Eigen::VectorXd x;
        double t;
        double dt;
        std::vector<Eigen::VectorXd> x_history;
        std::vector<double> t_history;

};

#endif
