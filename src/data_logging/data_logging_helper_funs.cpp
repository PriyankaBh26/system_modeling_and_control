# include <vector>
# include <cmath>
# include <iostream>
# include <Eigen/Dense>

# include "data_logging/data_logging_helper_funs.h"
# include "data_logging/savecsv.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

void SaveSimDataHistory(std::string directory, 
                        std::string problem, 
                        std::string data_type, 
                        std::vector<std::string> column_names, 
                        std::vector<VectorXd> x_history) {

    // save final output x to csv file
    std::string filename = directory + "/" + problem + "_" + data_type + ".csv";
    WriteMatToFile(filename, column_names, x_history);
    std::cout << "\n" + data_type + " size: " << x_history.size() << " " << x_history.size();
}


void SaveTimeHistory(std::string directory, 
                     std::string problem, 
                     std::vector<double> t_history) {

    // save final output x to csv file
    std::string filename = directory + "/" + problem + "_time.csv";
    WriteVecToFile(filename, {"time"}, t_history);
    std::cout << "\ntime history size: " << t_history.size();
}