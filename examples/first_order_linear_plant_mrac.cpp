# include <iostream>
# include <cmath>
# include <vector>
# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

# include "system_models/linear_time_invariant_system.h"
# include "data_logging/data_logging_helper_funs.h"
# include "controllers/controller_helper_funs.h"


VectorXd reference_traj(std::string traj_type, int num_states, double t) {
    VectorXd x_ref(1);
    if (traj_type == "step") {
        x_ref << 4;
    } else if (traj_type == "sine") {
        x_ref << 4 * sin(3 * t);
    }
    return x_ref;
}

VectorXd update_parameter(VectorXd ar, double signbp, double gamma, VectorXd error, VectorXd r, double dt) {
    VectorXd ardot(ar.size());
    ardot << -signbp * gamma * error * r;
    ar = ar + ardot * dt;
    return ar;
}

int main() {

    std::string soln_directory = "examples";
    std::string problem = "mrac_lti";

    // initialize state
    int num_states = 1; 
    VectorXd x0(num_states);
    x0 << 0.0;
    double t = 0.0;
    double dh = 1e-4;
    double dt = 1e-2;

    // initialize plant model
    MatrixXd Ap(num_states, num_states);
    Ap << 1;

    CheckSysStability(Ap, "Open Loop");

    MatrixXd Bp(num_states, 1);    
    Bp << 3;
    
    MatrixXd C(1, num_states);    
    C << 1;

    LinearTimeInvariantSys* plant = new LinearTimeInvariantSys(x0, t, dh, 
                                                                num_states, 
                                                                problem, Ap, Bp);
    std::cout << *plant;

    // initialize reference model
    MatrixXd Am(num_states, num_states);
    Am << -4;

    CheckSysStability(Am, "Open Loop");

    MatrixXd Bm(num_states, 1);    
    Bm << 4;

    LinearTimeInvariantSys* ref_model = new LinearTimeInvariantSys(x0, t, dh, 
                                                                num_states, 
                                                                "ref_model", Am, Bm);
    std::cout << *ref_model;

    // adaptive control parameters
    double gamma = 2; // adaptation gain

    std::string traj_type = "step";

    VectorXd x_ref = reference_traj(traj_type, num_states, t); 

    // initialize adaptive parameters
    VectorXd ar(num_states);
    ar << 0.5;
    VectorXd ax(num_states);
    ax << 1;

    double signbp = 1;

    VectorXd u(num_states);
    u << 0;
    VectorXd error(num_states);
    error << 0;

    int ode_timesteps = dt/dh;
    double t_final = 10;
    double measurement_noise = 0.01;

    // save x and t history
    std::vector<VectorXd> meas_history;
    std::vector<VectorXd> x_history;
    std::vector<VectorXd> x_ref_history;
    std::vector<VectorXd> error_history;
    std::vector<VectorXd> u_history;
    std::vector<double> t_history;

    meas_history.push_back(x0);
    x_history.push_back(x0);
    x_ref_history.push_back(x0);
    error_history.push_back(error);
    u_history.push_back(u);
    t_history.push_back(t);

    while (t < t_final) {
        plant->IntegrateODE(ode_timesteps, u);
        VectorXd x = plant->GetX();

        ref_model->IntegrateODE(ode_timesteps, x_ref);
        VectorXd x_m = ref_model->GetX();

        VectorXd z = x + measurement_noise * VectorXd::Random(num_states);
        meas_history.push_back(z);

        error = x - x_m;

        x_ref = reference_traj(traj_type, num_states, t); 
        ar = update_parameter(ar, signbp, gamma, error, x_ref, dt);
        ax = update_parameter(ax, signbp, gamma, error, x, dt);

        u = ar * x_ref + ax * x;

        t += dt;
        meas_history.push_back(z);
        x_history.push_back(x);
        x_ref_history.push_back(x_m);
        error_history.push_back(error);
        u_history.push_back(u);
        t_history.push_back(t);
    }

    // save final outputs to csv files
    SaveTimeHistory(soln_directory, problem, t_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "state_history", plant->GetColumnNames(), x_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "meas_history", plant->GetColumnNames(), meas_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "ref_history", plant->GetColumnNames(), x_ref_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "control_history", {"U0"}, u_history, "replace");
    SaveSimDataHistory(soln_directory, problem, "err_history", plant->GetColumnNames(), error_history, "replace");

    delete plant;
    delete ref_model;
    return 0;
}