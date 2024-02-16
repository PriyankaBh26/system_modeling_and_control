#ifndef DATA_LOGGING_HELPER_FUNS_H
#define DATA_LOGGING_HELPER_FUNS_H

# include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

void SaveSimDataHistory(std::string directory, 
                        std::string problem, 
                        std::string data_type, 
                        std::vector<std::string> column_names, 
                        std::vector<VectorXd> x_history,
                        std::string save_type);

void SaveTimeHistory(std::string directory, 
                     std::string problem, 
                     std::vector<double> t_history,
                     std::string save_type);
#endif