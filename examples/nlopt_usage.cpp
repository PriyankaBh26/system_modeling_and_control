#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <nlopt.hpp>

// the constraints are defined as C(x) <= 0

typedef struct {
    int count;
} counter;

double objective(const std::vector<double> &x, std::vector<double> &grad, void *data)
{   
    counter *d = reinterpret_cast<counter*>(data);
    ++d->count;
    if (!grad.empty()) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

typedef struct {
    double a, b;
} my_constraint_data;

double myconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    my_constraint_data *d = reinterpret_cast<my_constraint_data*>(data);
    double a = d->a, b = d->b;
    if (!grad.empty()) {
        grad[0] = 3 * a * std::pow((a*x[0] + b), 2);
        grad[1] = -1.0;
    }
    return std::pow((a*x[0] + b), 3) - x[1];
}

int main() {

    nlopt::opt opt(nlopt::LD_MMA, 2);

    std::vector<double> lb = {-HUGE_VAL, 0};
    opt.set_lower_bounds(lb);

    counter counter_data = {0};
    opt.set_min_objective(objective, &counter_data);

    my_constraint_data data[2] = { {2,0}, {-1,1} };
    opt.add_inequality_constraint(myconstraint, &data[0], 1e-8);
    opt.add_inequality_constraint(myconstraint, &data[1], 1e-8);

    opt.set_xtol_rel(1e-10);
    std::vector<double> x = {1.234, 5.678};
    double minf;

    try {
        nlopt::result result = opt.optimize(x, minf);
        std::cout << "found minimum after " << counter_data.count << " evaluations\n";
        std::cout << "result: " << result << "\n";
        std::cout << "found minimum at f(" << x[0] << "," << x[1] << ") = "
            << std::setprecision(10) << minf << std::endl;
    }
    catch(std::exception &e) {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

return 0;
}