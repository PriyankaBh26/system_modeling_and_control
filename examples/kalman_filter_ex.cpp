# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "controllers/pidcontroller.h"
# include "data_logging/savecsv.h"
# include "state_estimators/kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class solve_mass_spring_damper : public OdeSolver {
    public: 

        solve_mass_spring_damper(VectorXd y0, double t0, 
                                 double dt0, int num_states) : OdeSolver(y0, t0, dt0), 
                                 A(num_states, num_states) {};

        VectorXd f(double t, VectorXd y, VectorXd u) override {
            MatrixXd B(2,2);
            B(0,0) = 0;
            B(0,1) = 0;
            B(1,0) = 0;
            B(1,1) = 0;

            VectorXd yd = A * y + B * u;
            return yd;
        }

        MatrixXd GetA() {
            double k = 1.0;
            double c = 0.5;
            double m = 1.0;

            A(0,0) = 0;
            A(0,1) = 1;
            A(1,0) = -k/m;
            A(1,1) = -c/m;
            return A;
        }
        std::string GetName() override {
            return "msd_w_kalman_filter";
        }

        std::vector<std::string> GetColumnNames() override {
            return {"Pos", "Vel"};
        }

        private:
            MatrixXd A;

};

VectorXd CalculateXref(std::string reference_trajectory_type, int num_states, double t) {
    VectorXd x_ref(num_states);
    if (reference_trajectory_type == "step") {
        double A = 1.0;
        x_ref << A, 0.0;
    } else if (reference_trajectory_type == "sine") {
        double A = 1.0;
        double f = 1;
        x_ref << A*sin(2*M_PI*f*t), A*2*M_PI*f*cos(2*M_PI*f*t);
    } else if (reference_trajectory_type == "ramp") {
        double A = 1.0;
        double ramp_slope = 1.0;
        double max_ramp_time = 10;
        if (t < max_ramp_time) {
            x_ref << A*ramp_slope*t, A*ramp_slope;
        } else {
            x_ref << ramp_slope*max_ramp_time, 0;

        }
    }
    return x_ref;
}


void SaveSimulationData(OdeSolver* ode, std::vector<VectorXd>& x_history, std::vector<double> t_history, std::vector<VectorXd>& x_est_history, std::vector<VectorXd>& z_history) {

    // save final output x to csv file
    std::string filename = "examples/" + ode->GetName() + "_solution.csv";
    std::vector<std::string> column_names = ode->GetColumnNames();
    WriteMatToFile(filename, column_names, x_history);
    std::cout << x_history.size() << " " << x_history[0].size();

    // save estimated state history to csv file
    filename = "examples/" + ode->GetName() + "_meas_history.csv";
    WriteMatToFile(filename, column_names, z_history);

    // save estimated state history to csv file
    filename = "examples/" + ode->GetName() + "_est_history.csv";
    WriteMatToFile(filename, column_names, x_est_history);

    // save time to csv file
    filename = "examples/" + ode->GetName() + "_time.csv";
    column_names = {"time"};
    WriteVecToFile(filename, column_names, t_history);
}

int main() {
    int num_states = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 0.0, 1.0;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 1e-2, 0.0,
          0.0, 1e-2;

    // define mass-spring-damper system variables
    double t0 = 0.0;
    double dh = 1e-4;
    solve_mass_spring_damper* system = new solve_mass_spring_damper(x0, t0, dh, num_states);

    // define state matrix
    MatrixXd A = system->GetA();
    std::cout << "A =\n" << A << "\n";
    // define process noise matrix
    MatrixXd Q(num_states, num_states);
    Q << 1.0, 0.0,
          0.0, 1.0;
    // define output matrix
    MatrixXd H(num_states, num_states);
    H << 1.0, 0.0,
         0.0, 1.0;
    // define measurement noise matrix
    MatrixXd R(num_states, num_states);
    R << 1e-1, 0.0,
          0.0, 1e-2;

    // initialize kalman filter object
    VectorXd x0_m(num_states);
    x0_m << 0.0, 1.0;
    KalmanFilter* kf = new KalmanFilter(x0_m, P0, A, Q, H, R);

    // set integration duration
    double dt = 1e-2; 
    double time_final = 10.0;

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

        VectorXd x_est = kf->compute_estimate(z_history.back());

        t += dt;
        x_history.push_back(x);
        x_est_history.push_back(x_est);
        t_history.push_back(t);
    }
    // save simulation data for plotting
    SaveSimulationData(system, x_history, t_history, x_est_history, z_history);

    delete kf;
    return 0;
}