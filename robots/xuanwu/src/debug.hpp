#pragma once

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <Eigen/Dense>
#include <qpOASES.hpp>

#include <iomanip> // For setprecision

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

/**
 * Compares two Eigen matrices and reports specific element mismatches.
 * * @param A First matrix (e.g., "Correct Julia Matrix")
 * @param B Second matrix (e.g., "My C++ Matrix")
 * @param nameA Label for first matrix
 * @param nameB Label for second matrix
 * @param tolerance Threshold for considering two numbers different
 * @param max_print Maximum number of specific errors to print before silencing
 */
inline void compareMatrices(const Eigen::MatrixXd& A, 
                     const Eigen::MatrixXd& B, 
                     std::string nameA = "MatA", 
                     std::string nameB = "MatB", 
                     double tolerance = 1e-5,
                     int max_print = 20) 
{
    // 1. Check Dimensions
    if (A.rows() != B.rows() || A.cols() != B.cols()) {
        std::cerr << "[ERROR] Dimension Mismatch!\n"
                  << "  " << nameA << ": " << A.rows() << "x" << A.cols() << "\n"
                  << "  " << nameB << ": " << B.rows() << "x" << B.cols() << "\n";
        return;
    }

    std::cout << "\n>>> COMPARING " << nameA << " vs " << nameB << " <<<\n";
    std::cout << "    (Tolerance: " << tolerance << ")\n";

    int mismatch_count = 0;
    
    // 2. Iterate and Compare
    for (int i = 0; i < A.rows(); ++i) {
        for (int j = 0; j < A.cols(); ++j) {
            double valA = A(i, j);
            double valB = B(i, j);
            double diff = std::abs(valA - valB);

            if (diff > tolerance) {
                mismatch_count++;
                
                if (mismatch_count <= max_print) {
                    std::cout << std::setprecision(8) << std::scientific;
                    std::cout << "[MISMATCH] At (" << i << ", " << j << "):\n"
                              << "    " << nameA << ": " << valA << "\n"
                              << "    " << nameB << ": " << valB << "\n"
                              << "    Diff: " << diff << "\n"
                              << "-----------------------------\n";
                }
            }
        }
    }

    // 3. Final Summary
    if (mismatch_count == 0) {
        std::cout << ">>> SUCCESS: Matrices are identical (within tolerance).\n\n";
    } else {
        std::cout << ">>> FAILURE: Found " << mismatch_count << " total mismatches.\n";
        if (mismatch_count > max_print) {
            std::cout << "    (First " << max_print << " errors shown above)\n";
        }
        std::cout << "\n";
    }
}
