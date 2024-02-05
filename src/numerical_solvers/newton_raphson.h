#ifndef NEWTON_RAPHSON_H
#define NEWTON_RAPHSON_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class NewtonRaphson {
    public:
        NewtonRaphson(int num_states, int num_equations, VectorXd q0, double epsilon, int max_iterations);
        // the equation f(q) = 0
        virtual VectorXd f(VectorXd q);
        // the jacobian dfdq(q)
        virtual MatrixXd dfdq(VectorXd q);
        // iteration q = q0 - pinv(dfdq(q0)) * g(q0);
        VectorXd iterate();
        // save q history
        std::vector<VectorXd> get_q_history();
        // check for convergence and print success or failure message
        void convergence_check(double error, int num_iterations);

    private:
        int num_states;
        int num_equations;
        VectorXd q;
        double epsilon;
        int max_iterations;
        bool success;


};

#endif