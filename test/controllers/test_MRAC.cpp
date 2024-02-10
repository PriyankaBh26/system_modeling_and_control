# include <string>
# include <iostream>
# include <random>

# include <Eigen/Dense>

# include "controllers/model_reference_adaptive_controller.h"
# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/linear_time_invariant_system.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class NonlinearPlant : public OdeSolver {
    public:
        NonlinearPlant(VectorXd y0, double t0, double dt0, 
                        int n, std::string name,
                        MatrixXd A_in, MatrixXd B_in) : OdeSolver(y0, t0, dt0), 
                                                        num_states(n), sys_name(name),
                                                        dt(dt0), A(A_in), B(B_in) {};

        VectorXd f(double t, Eigen::VectorXd X, Eigen::VectorXd u) override {
            VectorXd nonlinear_fun(num_states); 
            VectorXd phi(6);
            phi << 1, X(0), X(1), X(0)*abs(X(0)), X(1)*abs(X(1)), std::pow(X(0),3);
            VectorXd w(6);
            w << 1, -1, 0.1, 0.1, 0.2, 0.1;

            nonlinear_fun << 0, (w.array() * phi.array()).sum();
            VectorXd Xdot = A * X + B * u + nonlinear_fun;

            X = X + Xdot * dt;

            return X;
        }
        std::string GetName() override {return sys_name;};

    private:
        int num_states;
        std::string sys_name;
        double dt;
        MatrixXd A;
        MatrixXd B;

};


int main() {
    int num_states = 2;

    // reference model matrices A_ref and B_ref
    MatrixXd A_ref(num_states, num_states);
    A_ref << 0 , 1,
            -4, -2;
    // Compute the eigenvalues of A_ref
    Eigen::EigenSolver<MatrixXd> solver(A_ref);
    Eigen::VectorXcd eigenvalues = solver.eigenvalues();
    std::cout << "A_ref eigenvalues:\n"  << eigenvalues;

    MatrixXd B_ref(1, num_states);
    B_ref << 0, 4;

    // initialize reference model ode solver
    VectorXd x0(num_states);
    double t0 = 0;
    double dh = 1e-4; 
    LinearTimeInvariantSys* ref_sys = new LinearTimeInvariantSys(x0, t0, dh, num_states, "ref_model", A_ref, B_ref);

    // nominal plant model matrices A and B
    MatrixXd A(num_states, num_states);
    A << 0, 1,
         0, 0;
    MatrixXd B(num_states, 1);
    B << 0, 1;

    // // initialize plant model ode solver
    NonlinearPlant* sys = new NonlinearPlant(x0, t0, dh, num_states, "plant", A, B);

    // solved Lyapunov equation using controller helper funs and A_ref
    MatrixXd P(num_states, num_states);
    P << 1.5, 0.125,
         0.125, 0.3125;

    int num_outputs = 1;

    // initialize MRAC model
    std::string disturbance_model_feature_type = "single_hidden_layer_nn";
    int num_features = 4;
    VectorXd disturbance_mean_std_bw(3);
    disturbance_mean_std_bw << 2,5,5;
    double dt = 0.01;
    double learning_rate_w = 10;
    double learning_rate_v = 10;
    double learning_rate_kx = 10;
    double learning_rate_kr = 10;

    DirectMRAC* mrac = new DirectMRAC(disturbance_model_feature_type,
                                      num_states,
                                      num_features,
                                      num_outputs,
                                      disturbance_mean_std_bw,
                                      dt,
                                      learning_rate_w,
                                      learning_rate_v,
                                      learning_rate_kx,
                                      learning_rate_kr,
                                      P,
                                      B);

    std::cout << *mrac;

    VectorXd x_hat(num_states+1);
    x_hat(0) = 1;
    x_hat.segment(1,num_states) = x0;

    VectorXd phi_x = mrac->phi(x_hat);
    std::cout << "\nphi(x) = \n" << phi_x.size();

    VectorXd sigmoid_x = mrac->sigmoid(x_hat);
    std::cout << "\nsigmoid(x) = \n" << sigmoid_x.size();

    MatrixXd dsigmoid_x = mrac->dsigmoid(x_hat);
    std::cout << "\ndsigmoid(x) = \n" << dsigmoid_x.size();

    mrac->UpdateWeights(x0); 

    MatrixXd w = mrac->GetW();
    std::cout << "\nw = \n" << w;

    MatrixXd V = mrac->GetV();
    std::cout << "\nV = \n" << V;

    mrac->UpdateKx(x0);
    VectorXd Kx = mrac->GetKx();
    std::cout << "\nKx = \n" << Kx;

    VectorXd r(num_outputs);
    
    mrac->UpdateKr(r);
    VectorXd Kr = mrac->GetKr();
    std::cout << "\nKr = \n" << Kr;





    delete ref_sys;
    delete sys;
    delete mrac;


    return 0;
}
