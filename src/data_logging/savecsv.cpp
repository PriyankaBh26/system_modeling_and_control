# include <iostream>
# include <fstream>
# include <string>
# include <filesystem>
# include <Eigen/Dense>

# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void WriteColumnNames(std::vector<std::string> column_names, std::ofstream output_file) {
    for (std::string const& col : column_names) {
        output_file << col << " ";
    }
    output_file << "\n";
}

void WriteToFile(std::vector<double> my_vec, std::ofstream output_file) {
    for (const auto& num : my_vec) {
        output_file << num << " ";
    }
    output_file << "\n";
}

void WriteMatToFile(std::string filename, 
                    std::vector<std::string> column_names, 
                    std::vector<VectorXd> my_vec,
                    std::string save_type) {
    namespace fs = std::filesystem;

    std::ofstream output_file;

    if (save_type == "append") {
        output_file.open(filename, std::ios::app);
    } else if (save_type == "replace") {
        output_file.open(filename, std::ios::trunc);
    }

    if (output_file.is_open()) {
        if (fs::is_empty(filename)) {
            for (std::string const& col : column_names) {
                output_file << col << " ";
            }
            output_file << "\n";
        }
        for (const auto& row : my_vec) {
            output_file << row.transpose() << "\n";
            }

        output_file.close();

        } else {
            std::cout << "Unable to open file.";
        }
    }

void WriteVecToFile(std::string filename, 
                    std::vector<std::string> column_names, 
                    std::vector<double> my_vec,
                    std::string save_type) {
    namespace fs = std::filesystem;

    std::ofstream output_file;

    if (save_type == "append") {
        output_file.open(filename, std::ios::app);
    } else if (save_type == "replace") {
        output_file.open(filename, std::ios::trunc);
    }

    if (output_file.is_open()) {
        if (fs::is_empty(filename)) {
            for (std::string const& col : column_names) {
                output_file << col << " ";
            }
            output_file << "\n";
        }
        for (const auto& num : my_vec) {
            output_file << num << "\n";
            }
            

        output_file.close();

        } else {
            std::cout << "Unable to open file.";
        }
    }


