# include <iostream>
# include <vector>
# include <Eigen/Dense>

# include "numerical_solvers/rk_ode_solver.h"
# include "numerical_solvers/solver_helper_funs.h"
# include "controllers/pidcontroller.h"
# include "data_logging/savecsv.h"
# include "state_estimators/kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class solve_mass_spring_damper : public OdeSolver {
    public: 

        solve_mass_spring_damper(VectorXd y0, double t0, double dt0, int num_states) : OdeSolver(y0, t0, dt0), A(num_states, num_states) {};

        VectorXd f(double t, VectorXd y, VectorXd u) override {
            double k = 1.0;
            double c = 1.0;
            double m = 1.0;

            A(0,0) = 0;
            A(0,1) = 1;
            A(1,0) = -k/m;
            A(1,1) = -c/m;

            MatrixXd B(2,2);
            B(0,0) = 0;
            B(0,1) = 0;
            B(1,0) = 1/m;
            B(1,1) = 1/m;

            VectorXd yd = A * y + B * u;
            return yd;
        }

        MatrixXd GetA() {
            return A;
        }
        std::string GetName() override {
            return "mass_spring_damper";
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

int main() {
    int num_states = 2;
    // initialize state vector
    VectorXd x0(num_states);
    x0 << 0.0, 0.0;
    // initialize error covariance matrix
    MatrixXd P0(num_states, num_states);
    P0 << 1e-3, 0.0,
          0.0, 1e-3;

    // define mass-spring-damper system variables
    double t0 = 0.0;
    double dh = 1e-4;
    solve_mass_spring_damper* system = new solve_mass_spring_damper(x0, t0, dh, num_states);

    // define state matrix
    MatrixXd A = system->GetA();

    // define process noise matrix
    MatrixXd Q(num_states, num_states);
    Q << 1e-3, 0.0,
          0.0, 1e-3;
    // define output matrix
    MatrixXd H(num_states, num_states);
    H << 1.0, 0.0,
         0.0, 1.0;
    // define measurement noise matrix
    MatrixXd R(num_states, num_states);
    R << 1e-3, 0.0,
          0.0, 1e-3;

    // initialize kalman filter object
    KalmanFilter* kf = new KalmanFilter(x0, P0, A, Q, H, R);

    VectorXd z(num_states);
    z << x0 + Q * x0;
    VectorXd x = kf->compute_estimate(z);

    delete kf;
    return 0;
}