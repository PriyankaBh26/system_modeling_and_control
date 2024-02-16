#ifndef NEWTON_RAPHSON_H
#define NEWTON_RAPHSON_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class NewtonRaphson {
    public:
        NewtonRaphson(int num_states, 
                      int num_equations, 
                      double tolerance, 
                      int max_iterations);
        virtual ~NewtonRaphson();
        
        // the equation f(q) = 0
        virtual VectorXd f(VectorXd q);
        // the jacobian dfdq(q)
        virtual MatrixXd dfdq(VectorXd q);
        // set initial value of q
        void SetInitialq(VectorXd q0);
        // iteration q = q0 - pinv(dfdq(q0)) * f(q0);
        VectorXd Iterate(VectorXd q0);
        // save q history
        std::vector<VectorXd> GetSolutionHistory();
        // check for convergence and print success or failure message
        void ConvergenceCheck(double error, int num_iterations);

    private:
        int num_states;
        int num_equations;
        VectorXd q;
        std::vector<VectorXd> q_history;
        double tolerance;
        int max_iterations;
        bool success;


};

#endif