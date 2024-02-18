# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "controllers/ackermans_formula_pole_placement.h"
# include "controllers/controller_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


void CheckSysStability(MatrixXd A, std::string system_type) {
    // Compute the eigenvalues of A
    Eigen::EigenSolver<MatrixXd> solver1(A);
    Eigen::VectorXcd evals = solver1.eigenvalues();
    std::cout << "\n" << system_type << " eigenvalues:\n"  << evals.transpose();
    int stable_eval = 0;
    for (const auto& eval : evals) {
        if (eval.real() > 0) {
            std::cout << "\n" << system_type << " System is unstable \n";
            break;
        } else if (eval.real() == 0) {
            std::cout << "\n" << system_type << " System is marginally stable \n";
            break;
        } else {
            stable_eval += 1;
        }
    }
    if (stable_eval == evals.size()) {
        std::cout << "\n" << system_type << " System is stable \n";
    }
}

VectorXd FindKAckermanFormula(MatrixXd A, MatrixXd B, VectorXd coeffs, std::string ip_vector_type) {
    VectorXd K;
    try {
        AckermansFormulaPolePlacement* system = new AckermansFormulaPolePlacement(ip_vector_type, coeffs, A, B);
        VectorXd K = system->AckermansFormula();
        CheckSysStability(A - B * K.transpose(), "CL");
        delete system;
        return K;
    }
    catch (const std::exception& e) {
        // Catch block to handle the exception
        std::cerr << "An exception occurred: " << e.what() << std::endl;
    }
    catch (const char* err_msg) {
        std::cout << "Exception: " << err_msg << std::endl;
    }
    return K;
}

VectorXd ScaleCLTransferFunction(MatrixXd A, MatrixXd B, MatrixXd C, VectorXd K, double r_ss) {
    int num_states = A.rows();
    int num_op = C.rows();

    MatrixXd AR(num_states+num_op,num_states+num_op);
    AR.block(0,0,num_states,num_states) = A;
    AR.block(0,num_states,num_states,1) = B;
    AR.block(num_states,0,num_op,num_states) = C;

    VectorXd b = VectorXd::Zero(num_states+num_op);
    b(num_states+num_op-1) = r_ss;

    VectorXd xu = AR.colPivHouseholderQr().solve(b);
    VectorXd Nx = xu.head(2).array() / r_ss;
    VectorXd Nu = xu.tail(1) / r_ss;

    VectorXd N_bar = Nu + K.transpose() * Nx;
    return N_bar;
}