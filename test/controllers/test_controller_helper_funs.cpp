# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "controllers/ackermans_formula_pole_placement.h"
# include "controllers/controller_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void TestCheckSysStability() {
    int num_states = 3;
    MatrixXd A(num_states, num_states);
    A << 1,2,3,
         2,4,6,
         1,0,9;

    std::string system_type = "OL";
    CheckSysStability(A, system_type);

}

void TestFindKAckermanFormula() {
    int num_states = 3;
    MatrixXd A(num_states, num_states);
    A << 1,2,3,
         2,3,5,
         1,0,9;

    MatrixXd B(num_states,1); 

    VectorXd coeffs(num_states+1);
    coeffs << 1, 7, 16, 12; 

    std::string ip_vector_type = "coeffs";

    VectorXd K = FindKAckermanFormula(A, B, coeffs, ip_vector_type);

}

void TestScaleCLTransferFunction() {
    int num_states = 2;
    MatrixXd A(num_states, num_states);
    A << 1, 1,
         1, 2;    
         
    MatrixXd B(num_states, 1);    
    B << 1, 0;
    
    MatrixXd C(1, num_states);    
    C << 1, 0;

    // coefficients of desired characteristic polynomial
    VectorXd coeffs(num_states+1);
    coeffs << 1, 4, 4;

    VectorXd K = FindKAckermanFormula(A, B, coeffs, "coeffs");
    
    double r_ss = 1;

    VectorXd N_bar = ScaleCLTransferFunction(A, B, C, K, r_ss);

}


int main() {
    
    TestCheckSysStability();

    TestFindKAckermanFormula();

    TestScaleCLTransferFunction();

    return 0;
}