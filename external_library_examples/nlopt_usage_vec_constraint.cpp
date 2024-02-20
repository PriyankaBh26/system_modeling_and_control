#include <nlopt.hpp>
#include <iostream>

double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data);

void multi_constraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data);

int main()
{   
    int num_x = 2;
    int num_constraints = 2;

    // initialize optimization object
	nlopt::opt opt(nlopt::LD_SLSQP, num_x);

    // set lower bounds
	std::vector<double> lb(num_x);
	lb[0] = -HUGE_VAL;   //HUGE_VAL is a C++ constant
	lb[1] = 0;
	opt.set_lower_bounds(lb);

	opt.set_min_objective(myfunc, NULL);

    // set fun arguments
	double constraint_args[4] = {2,0,-1,1};   //use one dimensional array

    // set constraint tolerances
	std::vector<double> tol_constraint(num_constraints);
	tol_constraint[0] = 1e-8;
	tol_constraint[1] = 1e-8;

    // add inequality constraints
	opt.add_inequality_mconstraint(multi_constraint, constraint_args, tol_constraint);

    // set convergence tolerance
	opt.set_xtol_rel(1e-4);

    // initialize x
	std::vector<double> x(num_x);
	x[0] = 1.234;
	x[1] = 5.678;

    // initialize cost 
	double minf;

    // solve optimization problem and store the result
	nlopt::result result = opt.optimize(x, minf);

    // print the result
	std::cout << "The result is" << std::endl;
	std::cout << result << std::endl;
	std::cout << "Minimal function value " << minf << std::endl;
}

double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    if (!grad.empty()) {
        grad[0] = 0.0;
        grad[1] = 0.5 / sqrt(x[1]);
    }
    return sqrt(x[1]);
}

void multi_constraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* f_data)
{
    //n is the length of x, m is the length of result
    double *df_data = static_cast<double*>(f_data);
    double a1 = df_data[0];
    double b1 = df_data[1];
    double a2 = df_data[2];
    double b2 = df_data[3];

    //The n dimension of grad is stored contiguously, so that \partci/\partxj is stored in grad[i*n + j]
    //Here you see take dCi/dx0...dxn and store it one by one, then repeat. grad is just an one dimensional array

    if (grad) {
        grad[0] = 3 * a1 * (a1*x[0] + b1) * (a1*x[0] + b1);
        grad[1] = -1.0;
        grad[2] = 3 * a2 * (a2*x[0] + b2) * (a2*x[0] + b2);
        grad[3] = -1.0;
    }

    result[0] = ((a1*x[0] + b1) * (a1*x[0] + b1) * (a1*x[0] + b1) - x[1]);
    result[1] = ((a2*x[0] + b2) * (a2*x[0] + b2) * (a2*x[0] + b2) - x[1]);

    return;
}