#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <Eigen/Dense>
#include <qpOASES.hpp>

/**
 * Dumps a flattened std::vector (qpOASES raw data) to a text file.
 * Useful for inspecting H, A, g, lb, ub, etc.
 */
inline void dumpRawData(const std::string& filename, 
                        const std::vector<qpOASES::real_t>& data, 
                        int rows, int cols) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[Debug] Error: Could not open " << filename << " for writing." << std::endl;
        return;
    }

    // High precision to catch numerical issues
    file << std::scientific << std::setprecision(9);

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            int idx = i * cols + j;
            if (idx < data.size()) {
                file << data[idx] << " ";
            } else {
                file << "ERR "; // Safety for index out of bounds
            }
        }
        file << "\n";
    }
    file.close();
    std::cout << "[Debug] Saved raw data to: " << filename << std::endl;
}

/**
 * Dumps an Eigen Matrix or Vector to a text file using Eigen's formatter.
 */
inline void dumpEigenData(const std::string& filename, 
                          const Eigen::MatrixXd& mat) 
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[Debug] Error: Could not open " << filename << " for writing." << std::endl;
        return;
    }
    
    // Format: Full Precision, space-separated, newlines strictly handled
    Eigen::IOFormat CleanFmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "", "");
    
    file << mat.format(CleanFmt);
    file.close();
    std::cout << "[Debug] Saved Eigen data to: " << filename << std::endl;
}
