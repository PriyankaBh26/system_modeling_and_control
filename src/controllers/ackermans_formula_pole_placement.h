#ifndef ACKERMANS_FORMULA_POLE_PLACEMENT_H
#define ACKERMANS_FORMULA_POLE_PLACEMENT_H

# include <string>
# include <iostream>
# include <Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXcd;

class AckermansFormulaPolePlacement {
    public:
        AckermansFormulaPolePlacement(std::string ip_vector_type, VectorXd ip_vec, MatrixXd A, MatrixXd B);

        // eigenvalues(desired_A - K*A) = desired_roots

        // Ackerman's formula
        // K = [0 0 .. 1] * C.inv() * delta(A);
        MatrixXd CharacteristicPolynomial();

        MatrixXd ControllabilityMatrix();

        VectorXd AckermansFormula();

        MatrixXd CalculateResultantA();

        VectorXcd CalculateEigenValues(MatrixXd ip_mat); 

        friend std::ostream& operator << (std::ostream& out, AckermansFormulaPolePlacement& system);

    private:
        std::string ip_vector_type;
        MatrixXd A;
        MatrixXd B;
        int n;
        VectorXd K;
        VectorXd desired_roots;
        VectorXd polynomial_coefficients;
};

#endif