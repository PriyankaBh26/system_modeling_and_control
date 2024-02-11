# include <vector>
# include <string>
# include <cmath>
# include <Eigen/Dense>

# include "controllers/ackermans_formula_pole_placement.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

int main() {
    int num_states = 2;
    MatrixXd A(num_states, num_states);
    A << 1, 1,
         1, 2;
    MatrixXd B(num_states, 1);    
    B << 1, 0;

    VectorXd desired_roots(num_states);
    desired_roots << -6, -5;

    VectorXd coeffs(num_states+1);
    coeffs << 1, 11, 30;

    // AckermansFormulaPolePlacement* system = new AckermansFormulaPolePlacement("roots", desired_roots, A, B);
    AckermansFormulaPolePlacement* system = new AckermansFormulaPolePlacement("coeffs", coeffs, A, B);

    MatrixXd delta_A = system->CharacteristicPolynomial();
    MatrixXd exp_delta_A(num_states, num_states);
    exp_delta_A << 43, 14,
                   14, 57;
    MatrixXd err_delta_A = exp_delta_A - delta_A;
    std::cout << "\nerror in delta_A: \n" << err_delta_A;

    MatrixXd C = system->ControllabilityMatrix();
    MatrixXd exp_C_inv(num_states, num_states);
    exp_C_inv << 1, -1,
             0, 1;
            
    MatrixXd err_C_inv = exp_C_inv - C.inverse();
    std::cout << "\nerror in C_inverse: \n" << err_C_inv;

    VectorXd K = system->AckermansFormula();
    VectorXd exp_K(num_states);
    exp_K << 14, 57;

    MatrixXd err_K = exp_K - K;
    std::cout << "\nerror in K: \n" << err_K;

    MatrixXd A_resultant = system->CalculateResultantA();
    VectorXcd eigenvalues = system->CalculateEigenValues(A_resultant);
    std::cout << "\nResultant eignevalues are: \n" << eigenvalues;

    std::cout << *system;

    delete system;

    return 0;
}