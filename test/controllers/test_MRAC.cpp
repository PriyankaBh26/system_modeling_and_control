# include <string>
# include <iostream>
# include <random>

# include <Eigen/Dense>

# include "controllers/model_reference_adaptive_controller.h"
# include "numerical_solvers/rk_ode_solver.h"
# include "system_models/linear_time_invariant_system.h"
# include "data_logging/data_logging_helper_funs.h"

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

void test_direct_mrac_functions(DirectMRAC* mrac, VectorXd x0, VectorXd r) {
    VectorXd x_hat = mrac->CalculateXhat(x0);

    VectorXd phi_x = mrac->phi(x_hat);
    std::cout << "\nphi(x) = \n" << phi_x.size();

    VectorXd sigmoid_x = mrac->sigmoid(x_hat);
    std::cout << "\nsigmoid(x) = \n" << sigmoid_x.size();

    MatrixXd dsigmoid_x = mrac->dsigmoid(x_hat);
    std::cout << "\ndsigmoid(x) = \n" << dsigmoid_x.size();

    mrac->UpdateWeights(x_hat); 

    MatrixXd w = mrac->GetW();
    std::cout << "\nw = \n" << w;

    MatrixXd V = mrac->GetV();
    std::cout << "\nV = \n" << V;

    mrac->UpdateKx(x0);
    VectorXd Kx = mrac->GetKx();
    std::cout << "\nKx = \n" << Kx;

    mrac->UpdateKr(r);
    VectorXd Kr = mrac->GetKr();
    std::cout << "\nKr = \n" << Kr;

    mrac->CalculateError(x0, x0);
    VectorXd error = mrac->GetError();
    std::cout << "\nerror = \n" << error;

    VectorXd u_ad = mrac->UpdateAdaptiveControlInput(x0);
    std::cout << "\nu_ad = \n" << u_ad;

    VectorXd u = mrac->UpdateControlInput(r, x0, x0);
    std::cout << "\nu = \n" << u;

    std::vector<VectorXd> err_history = mrac->GetErrorHistory();
    std::cout << "\nerr_history = \n" << err_history[0];

    std::vector<VectorXd> u_history = mrac->GetControlInputHistory();
    std::cout << "\nu_history = \n" << u_history[0];

    std::cout << "\ntesting of MRAC functions Complete!\n";
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

    MatrixXd B_ref(num_states,1);
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
    VectorXd u(num_outputs);

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

    // initialize reference input
    VectorXd r(num_outputs);
    r << 1.0;

    // test mrac functions
    // test_direct_mrac_functions(mrac, x0, r);

     // initialize measured output z
    std::vector<VectorXd> meas_history;
    double measurement_noise = 0.001;
    // initialize and set time parameters
    double t = 0;
    double time_final = 1;

    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_ref_history;
    std::vector<double> t_history;
    x_history.push_back(x0);
    x_ref_history.push_back(r);
    t_history.push_back(t);

    // solve mrac with ode integration
    while (t < time_final) {
        int ode_timesteps = dt/dh;
        sys->IntegrateODE(ode_timesteps, u);
        ref_sys->IntegrateODE(ode_timesteps, r);

        VectorXd x = sys->GetX();

        // std::cout << "\nx =\n " << x;

        VectorXd x_ref = ref_sys->GetX();

        // std::cout << "\nx_ref =\n " << x_ref;

        meas_history.push_back(x + measurement_noise * VectorXd::Random(num_states));

        u = mrac->UpdateControlInput(r, x_ref, x);

        t += dt;
        x_ref_history.push_back(r);
        x_history.push_back(x);
        t_history.push_back(t);
    }

    // save simulation data
    std::string directory = "test/controllers";
    std::string problem = "mrac";
    SaveTimeHistory(directory, problem, t_history);
    SaveSimDataHistory(directory, problem, "state_history", sys->GetColumnNames(), x_history);
    SaveSimDataHistory(directory, problem, "meas_history", sys->GetColumnNames(), meas_history);
    SaveSimDataHistory(directory, problem, "control_history", {"U1"}, mrac->GetControlInputHistory());
    SaveSimDataHistory(directory, problem, "err_history", sys->GetColumnNames(), mrac->GetErrorHistory());

    delete ref_sys;
    delete sys;
    delete mrac;


    return 0;
}
