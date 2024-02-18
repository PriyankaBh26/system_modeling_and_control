# include <vector>
# include <string>
# include <cmath>
# include <Eigen/Dense>

# include "controllers/ackermans_formula_pole_placement.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXcd;

AckermansFormulaPolePlacement::AckermansFormulaPolePlacement(std::string str, 
                                                            VectorXd ip_vec, 
                                                            MatrixXd A_in,
                                                            MatrixXd B_in) : ip_vector_type(str), 
                                                                            A(A_in),
                                                                            B(B_in) {
    n = A.rows();
    VectorXd K(n,1);
    if (ip_vector_type == "roots") {
        desired_roots = ip_vec;
    } else if (ip_vector_type == "coeffs") {
        polynomial_coefficients = ip_vec;
    }
    
};


MatrixXd AckermansFormulaPolePlacement::CharacteristicPolynomial() {
    MatrixXd delta_A = MatrixXd::Identity(n, n);

    if (ip_vector_type == "roots") {
        for (const auto& root : desired_roots) {
            delta_A = delta_A * (A - root * MatrixXd::Identity(n, n));
        }
    } else if (ip_vector_type == "coeffs") {
        MatrixXd A_pow = MatrixXd::Identity(n, n);
        delta_A = delta_A * polynomial_coefficients(n);
        for (int i{1}; i<=n; i++) {
            A_pow = A_pow * A;
            delta_A = delta_A + A_pow * polynomial_coefficients(n - i);
        }
    }
    return delta_A;
};

MatrixXd AckermansFormulaPolePlacement::ControllabilityMatrix() {
    MatrixXd C(n, n);
    MatrixXd A_pow = MatrixXd::Identity(n, n);
    for (int i{0}; i<n; i++) {
        C.block(0,i,n,1) = A_pow * B;
        A_pow = A_pow * A;
    }
    return C;
};

bool AckermansFormulaPolePlacement::CheckSystemControllability(MatrixXd C) {
    Eigen::FullPivLU<MatrixXd> lu_decomp(C);
    int rank = lu_decomp.rank();
    bool controllability = false;
    if (rank == n) {
        controllability = true;
    }
    return controllability;
}

VectorXd AckermansFormulaPolePlacement::AckermansFormula() {
    VectorXd one_n(n);
    one_n(n-1) = 1;
    std::cout << "\none_n:\n" << one_n;
    MatrixXd C = AckermansFormulaPolePlacement::ControllabilityMatrix();

    bool controllability = AckermansFormulaPolePlacement::CheckSystemControllability(C);
    if (controllability) {
        MatrixXd delta_A = AckermansFormulaPolePlacement::CharacteristicPolynomial();
        // K = [0 0 .. 1] * C.inv() * delta(A);
        K = one_n.transpose() * C.inverse() * delta_A;
    } else {
        std::cout << "system is not controllable."
        throw 0;
    }
    return K.transpose();

};

MatrixXd AckermansFormulaPolePlacement::CalculateResultantA() {
    MatrixXd resultant_A(n,n);
    resultant_A = A - B * K.transpose();
    return resultant_A;
}

VectorXcd AckermansFormulaPolePlacement::CalculateEigenValues(MatrixXd ip_mat) {
    // Compute the eigenvalues
    Eigen::EigenSolver<MatrixXd> solver(ip_mat);
    VectorXcd eigenvalues = solver.eigenvalues();
    return eigenvalues;
}

std::ostream& operator << (std::ostream& out, AckermansFormulaPolePlacement& system) {

    out << "\nPrinting Feedback gains for given system using Ackerman's formula:\n";

    if (system.ip_vector_type == "roots") {
        out << "desired roots: \n";
        out << system.desired_roots << "\n";

    } else if (system.ip_vector_type == "coeffs") {
        out << "desired coefficients: \n";
        out << system.polynomial_coefficients << "\n";
    }
        out << "(A) State matrix: \n";
        out << system.A << "\n";
        out << "(B) Input Vector: \n";
        out << system.B << "\n";
        out << "(K) Feedback gain vector: \n";
        out << system.K << "\n";
    return out;
};

