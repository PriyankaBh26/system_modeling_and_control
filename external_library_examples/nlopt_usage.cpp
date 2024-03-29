#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <nlopt.hpp>

// the constraints are defined as C(x) <= 0

typedef struct {
    int count;
} obj_arguments;

double objective(const std::vector<double> &x, std::vector<double> &grad, void *data)
{   
    obj_arguments *d = reinterpret_cast<obj_arguments*>(data);
    ++d->count;
    if (!grad.empty()) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

typedef struct {
    double a, b;
} constraint_arguments;

double myconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    constraint_arguments *d = reinterpret_cast<constraint_arguments*>(data);
    double a = d->a, b = d->b;
    if (!grad.empty()) {
        grad[0] = 3 * a * std::pow((a*x[0] + b), 2);
        grad[1] = -1.0;
    }
    return std::pow((a*x[0] + b), 3) - x[1];
}

int main() {

    int x_length = 2;
    nlopt::opt opt(nlopt::LD_MMA, x_length);

    std::vector<double> lb = {-HUGE_VAL, 0};
    opt.set_lower_bounds(lb);

    obj_arguments obj_args = {0};
    opt.set_min_objective(objective, &obj_args);

    constraint_arguments constraint_args[2] = { {2,0}, {-1,1} };
    opt.add_inequality_constraint(myconstraint, &constraint_args[0], 1e-8);
    opt.add_inequality_constraint(myconstraint, &constraint_args[1], 1e-8);

    opt.set_xtol_rel(1e-10);
    std::vector<double> x = {1.234, 5.678};
    double minf;

    try {
        nlopt::result result = opt.optimize(x, minf);
        std::cout << "found minimum after " << obj_args.count << " evaluations\n";
        std::cout << "result: " << result << "\n";
        std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
            << std::setprecision(10) << minf << std::endl;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

return 0;
}