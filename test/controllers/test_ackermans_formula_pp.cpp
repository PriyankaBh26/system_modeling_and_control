# include <vector>
# include <string>
# include <cmath>
# include <Eigen/Dense>

# include "controllers/ackermans_formula_pole_placement.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void TestCharacteristicPolynomial(AckermansFormulaPolePlacement* system, int num_states) {
    MatrixXd delta_A = system->CharacteristicPolynomial();
    MatrixXd exp_delta_A(num_states, num_states);
    exp_delta_A << 43, 14,
                   14, 57;
    MatrixXd err_delta_A = exp_delta_A - delta_A;

    std::cout << "\n TestCharacteristicPolynomial \n";
    
    if (((err_delta_A).array().abs() < 1e-5).all()) {
        std::cout << "\ntest successful!\n";
    } else {
        std::cout << "\ntest failed!\n";
    }
    std::cout << "\n";
}

MatrixXd TestControllabilityMatrix(AckermansFormulaPolePlacement* system, int num_states) {
    MatrixXd C = system->ControllabilityMatrix();
    MatrixXd exp_C_inv(num_states, num_states);
    exp_C_inv << 1, -1,
             0, 1;

    MatrixXd err_C_inv = exp_C_inv - C.inverse();

    std::cout << "\n TestControllabilityMatrix \n";
    
    if (((err_C_inv).array().abs() < 1e-5).all()) {
        std::cout << "\ntest successful!\n";
    } else {
        std::cout << "\ntest failed!\n";
    }
    std::cout << "\n";
    return C;
}

void TestCheckSystemControllability(AckermansFormulaPolePlacement* system, int num_states, MatrixXd C) {

    bool controllability = system->CheckSystemControllability(C);
    
    std::cout << "\n TestCheckSystemControllability \n";
    
    if (controllability == 1) {
        std::cout << "\ntest successful!\n";
    } else {
        std::cout << "\ntest failed!\n";
    }
    std::cout << "\n";
}

void TestAckermansFormula(AckermansFormulaPolePlacement* system, int num_states) {
    VectorXd K = system->AckermansFormula();
    VectorXd exp_K(num_states);
    exp_K << 14, 57;

    MatrixXd err_K = exp_K - K;

    std::cout << "\n TestAckermansFormula \n";
    
    if (((err_K).array().abs() < 1e-5).all()){
        std::cout << "\ntest successful!\n";
    } else {
        std::cout << "\ntest failed!\n";
    }
    std::cout << "\n";

}

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

    TestCharacteristicPolynomial(system, num_states);

    MatrixXd C = TestControllabilityMatrix(system, num_states);

    TestCheckSystemControllability(system, num_states, C);

    TestAckermansFormula(system, num_states);

    MatrixXd A_resultant = system->CalculateResultantA();
    VectorXcd eigenvalues = system->CalculateEigenValues(A_resultant);
    std::cout << "\nResultant eignevalues are: \n" << eigenvalues;

    std::cout << *system;

    delete system;

    return 0;
}