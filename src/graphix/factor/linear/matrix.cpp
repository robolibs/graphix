#include "graphix/factor/linear/matrix.hpp"
#include <cmath>
#include <stdexcept>

namespace graphix::factor {

    std::vector<double> Matrix::solve_cholesky(const std::vector<double> &b) const {
        if (rows_ != cols_) {
            throw std::invalid_argument("Matrix must be square for Cholesky decomposition");
        }
        if (b.size() != rows_) {
            throw std::invalid_argument("Vector size must match matrix size");
        }

        size_t n = rows_;

        // Perform Cholesky decomposition: A = L * L^T
        // L is lower triangular
        Matrix L(n, n);

        for (size_t i = 0; i < n; ++i) {
            for (size_t j = 0; j <= i; ++j) {
                double sum = 0.0;

                if (j == i) {
                    // Diagonal elements
                    for (size_t k = 0; k < j; ++k) {
                        sum += L(j, k) * L(j, k);
                    }

                    double diag_val = (*this)(j, j) - sum;
                    if (diag_val <= 0.0) {
                        throw std::runtime_error("Matrix is not positive definite (Cholesky failed)");
                    }
                    L(j, j) = std::sqrt(diag_val);
                } else {
                    // Non-diagonal elements
                    for (size_t k = 0; k < j; ++k) {
                        sum += L(i, k) * L(j, k);
                    }
                    L(i, j) = ((*this)(i, j) - sum) / L(j, j);
                }
            }
        }

        // Forward substitution: solve L * y = b
        std::vector<double> y(n);
        for (size_t i = 0; i < n; ++i) {
            double sum = 0.0;
            for (size_t j = 0; j < i; ++j) {
                sum += L(i, j) * y[j];
            }
            y[i] = (b[i] - sum) / L(i, i);
        }

        // Backward substitution: solve L^T * x = y
        std::vector<double> x(n);
        for (int i = n - 1; i >= 0; --i) {
            double sum = 0.0;
            for (size_t j = i + 1; j < n; ++j) {
                sum += L(j, i) * x[j]; // L^T(i,j) = L(j,i)
            }
            x[i] = (y[i] - sum) / L(i, i);
        }

        return x;
    }

} // namespace graphix::factor
