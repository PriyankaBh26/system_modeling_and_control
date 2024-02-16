# include <iostream>
# include <fstream>
# include <string>
# include <filesystem>
# include <Eigen/Dense>

# include "data_logging/savecsv.h"

std::vector<Eigen::VectorXd> CreateMatrix() {
    std::vector<Eigen::VectorXd> my_mat;
    Eigen::VectorXd vec(3);

    for (int i{0}; i < 10; i++) {
        double A = i + 5.56;
        double B = i * 10.98;
        double C = i - 10;
        
        vec << A, B, C;
        my_mat.push_back(vec);
    }
    return my_mat;
}

int main () {

    std::string filename = "examples/create_csv_ex_1.csv";

    std::vector<std::string> column_names = {"A", "B", "C"};

    std::vector<Eigen::VectorXd> my_mat = CreateMatrix();
    
    WriteMatToFile(filename, column_names, my_mat, "replace");

    return 0;
}