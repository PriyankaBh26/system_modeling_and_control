# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "controllers/pidcontroller.h"
# include "data_logging/savecsv.h"
# include "data_logging/data_logging_helper_funs.h"
# include "state_estimators/extended_kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

static const double MU = 2.5;

class VanDerPolOscillator : public OdeSolver {
    public: 

        VanDerPolOscillator(VectorXd x0, double t0, double dt0) : OdeSolver(x0, t0, dt0) {};

        VectorXd f(double t, VectorXd x, VectorXd u) override {
            
            VectorXd xd(2);
            xd << x[1] + u[0], MU * (1 - std::pow(x[0], 2)) * x[1] - x[0] + u[1];

            return xd;
        }

        std::string GetName() override {
            return "van_der_pol";
        }

        std::vector<std::string> GetColumnNames() override {
            return {"Pos", "Vel"};
        }
};

class EKF : public ExtendedKalmanFilter {

    public:
    EKF(VectorXd x0, MatrixXd P0, MatrixXd Q_in, 
        MatrixXd R_in, double dt, int n_in, int m_in) : ExtendedKalmanFilter(x0, P0, Q_in, R_in, dt, n_in, m_in) {};

    // update state variables, f(x) = [x1dot, x2dot .. xndot]T
    VectorXd f() override {
        int n = GetN();
        double dt = GetDt();
        VectorXd x = GetX();
        MatrixXd xdot(n, 1);
        xdot << x[1], MU * (1 - std::pow(x[0], 2)) * x[1] - x[0];
        
        x = x + xdot*dt;
        return x;
    };

    // update outputs, y = h(x)
    VectorXd h() override {
        int n = GetN();
        int m = GetM();
        VectorXd x = GetX();
        VectorXd y(n);
        MatrixXd H(m, n);
        H << 1, 0,
            0, 1;
        y = H * x;
        return y;
    };

    // calculate f jacobian = [df/dx1 df/dx2 .. df/dxn]
    MatrixXd calculate_f_jacobian() override {
        int n = GetN();
        double dt = GetDt();
        VectorXd x = GetX();

        MatrixXd A(n, n);
        A << 0, 1, 
            MU * 2 * x[0] * x[1] - 1, MU *  (1 - std::pow(x[0], 2));
        A = MatrixXd::Identity(n, n) + A * dt;
        return A;
    };

    // calculate h jacobian H 
    MatrixXd calculate_h_jacobian() override {
        int n = GetN();
        int m = GetM();
        MatrixXd H(m, n);
        H << 1, 0,
            0, 1;
        return H;
    };

};

int main() {
    int num_states = 2;
    int num_outputs = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 1.0, 1.0;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 10, 0.0,
          0.0, 10;

    // define mass-spring-damper system variables
    double t0 = 0.0;
    double dh = 1e-4;
    VanDerPolOscillator* system = new VanDerPolOscillator(x0, t0, dh);

    // set integration duration
    double dt = 1e-2; 
    double time_final = 10.0;

    // define process noise matrix
    MatrixXd Q(num_states, num_states);
    Q << 1e-2, 0.0,
          0.0, 1e-2;
    // define measurement noise matrix
    MatrixXd R(num_states, num_outputs);
    R << 1e-1, 0.0,
          0.0, 1e-2;

    // initialize kalman filter object
    VectorXd x0_m(num_states);
    x0_m << 0.0, 0.0;
    EKF* ekf = new EKF(x0_m, P0, Q, R, dt, num_states, num_outputs);

    // initialize measurement z
    std::vector<VectorXd> z_history;

    // initialize control input
    VectorXd u(num_states);

    // save x and t history
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_est_history;
    std::vector<double> t_history;

    // system dynamics and controller action
    double t = 0;
    while (t < time_final) {
        int ode_timesteps = dt/dh;
        system->IntegrateODE(ode_timesteps, u);

        VectorXd x = system->GetX();
        
        z_history.push_back(x + R * VectorXd::Random(num_states));

        VectorXd x_est = ekf->compute_estimate(z_history.back());

        t += dt;
        x_history.push_back(x);
        x_est_history.push_back(x_est);
        t_history.push_back(t);
    }
    // save simulation data for plotting
    SaveKFSimulationData(system, x_history, t_history, x_est_history, z_history);

    delete ekf;
    delete system;
    return 0;
}