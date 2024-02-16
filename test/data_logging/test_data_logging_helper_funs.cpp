# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "data_logging/data_logging_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

std::vector<VectorXd> CreateMatrix() {
    std::vector<VectorXd> my_mat;
    VectorXd vec(3);

    for (int i{0}; i < 10; i++) {
        double A = i + 5.56;
        double B = i * 10.98;
        double C = i - 10;
        
        vec << A, B, C;
        my_mat.push_back(vec);
    }
    return my_mat;
}

std::vector<double> CreateVector() {
    std::vector<double> my_vec;
    for (int i{0}; i < 10; i++) {
        double A = i + 5.56;
        my_vec.push_back(A);
    }
    return my_vec;
}

void TestSaveSimDataHistory() {
    std::string directory = "test/data_logging";
    std::string problem = "test_data_logging";
    std::string data_type = "matrix";
    std::vector<std::string> column_names = {"A", "B", "C"};
    std::vector<VectorXd> my_mat = CreateMatrix();

    SaveSimDataHistory(directory,
                        problem,
                        data_type,
                        column_names,
                        my_mat,
                        "replace");

}

void TestSaveTimeHistory() {
    std::string directory = "test/data_logging";
    std::string problem = "test_data_logging"; 
    std::vector<double> t_history = CreateVector();

    SaveTimeHistory(directory, 
                    problem, 
                    t_history,
                    "append");

}

int main() {
    TestSaveSimDataHistory();

    TestSaveTimeHistory();

    return 0;
}