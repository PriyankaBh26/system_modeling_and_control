#ifndef SAVECSV_H
#define SAVECSV_H

# include <Eigen/Dense>

void WriteColumnNames(std::vector<std::string> column_names, std::ofstream output_file);

void WriteToFile(std::vector<double> my_vec, std::ofstream output_file);

void WriteMatToFile(std::string filename, std::vector<std::string> column_names, std::vector<Eigen::VectorXd> my_mat, std::string save_type);

void WriteVecToFile(std::string filename, std::vector<std::string> column_names, std::vector<double> my_vec, std::string save_type);

#endif // SAVECSV_H