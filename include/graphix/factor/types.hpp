#pragma once

/**
 * @file factor/types.hpp
 * @brief Type definitions for the factor graph module
 *
 * Architecture:
 * - DATAPOD owns all data (dp::mat::vector, dp::mat::matrix, dp::Vector, etc.)
 * - OPTINUM provides SIMD operations on datapod types (does NOT own data)
 *
 * This file provides:
 * - Type aliases for common matrix/vector types used in factor graphs
 * - Helper functions for matrix operations needed by optimizers
 *
 * IMPORTANT: datapod matrices use COLUMN-MAJOR storage layout.
 */

#include <datapod/associative.hpp>
#include <datapod/matrix.hpp>
#include <datapod/sequential.hpp>
#include <datapod/trees.hpp>

#include <optinum/lie/groups/se2.hpp>
#include <optinum/lina/solve/solve_dynamic.hpp>
#include <optinum/simd/matrix.hpp>
#include <optinum/simd/vector.hpp>

namespace graphix::factor {

    namespace dp = ::datapod;
    namespace on = ::optinum;

    // =========================================================================
    // Fixed-size vector types (data storage from datapod::mat)
    // =========================================================================

    using Vec3d = dp::mat::vector3d;
    using Vec3f = dp::mat::vector3f;

    // =========================================================================
    // Lie group types (from optinum::lie)
    // =========================================================================

    using SE2d = on::lie::SE2<double>;
    using SE2f = on::lie::SE2<float>;
    using SO2d = on::lie::SO2<double>;
    using SO2f = on::lie::SO2<float>;

    // =========================================================================
    // Dynamic-size vector types (data storage from datapod::mat)
    // =========================================================================

    using DynVec = dp::mat::VectorXd;
    using DynVecF = dp::mat::VectorXf;

    // =========================================================================
    // Dynamic-size matrix types (data storage from datapod::mat, COLUMN-MAJOR)
    // =========================================================================

    using DynMat = dp::mat::MatrixXd;
    using DynMatF = dp::mat::MatrixXf;

    // =========================================================================
    // Fixed-size matrix types (data storage from datapod::mat, COLUMN-MAJOR)
    // =========================================================================

    using Mat3d = dp::mat::matrix3x3d;
    using Mat3f = dp::mat::matrix3x3f;

    // =========================================================================
    // Container types (from datapod)
    // =========================================================================

    template <typename T> using Vector = dp::Vector<T>;

    template <typename K, typename V> using Map = dp::Map<K, V>;

    template <typename T> using Set = dp::Set<T>;

    template <typename K, typename V> using OrderedMap = dp::OrderedMap<K, V>;

    // =========================================================================
    // SIMD view types (non-owning, for operations)
    // =========================================================================

    using DynVecView = on::simd::Vector<double, on::simd::Dynamic>;
    using DynVecViewF = on::simd::Vector<float, on::simd::Dynamic>;
    using DynMatView = on::simd::Matrix<double, on::simd::Dynamic, on::simd::Dynamic>;
    using DynMatViewF = on::simd::Matrix<float, on::simd::Dynamic, on::simd::Dynamic>;

    // =========================================================================
    // Helper functions for matrix operations
    // =========================================================================

    /**
     * @brief Solve linear system Ax = b using LU decomposition
     *
     * @param a Coefficient matrix (n x n) - datapod storage
     * @param b Right-hand side vector (n elements) - datapod storage
     * @return Solution vector x - datapod storage
     */
    inline DynVec solve(const DynMat &a, const DynVec &b) {
        // Create non-owning views over the datapod storage
        DynMatView a_view(const_cast<DynMat &>(a));
        DynVecView b_view(const_cast<DynVec &>(b));

        // Use optinum's dynamic solver
        return on::lina::solve_dynamic<double>(a_view, b_view);
    }

    /**
     * @brief Try to solve linear system Ax = b with error handling
     *
     * @param a Coefficient matrix (n x n)
     * @param b Right-hand side vector (n elements)
     * @return Result containing solution or error
     */
    inline dp::Result<DynVec, dp::Error> try_solve(const DynMat &a, const DynVec &b) {
        DynMatView a_view(const_cast<DynMat &>(a));
        DynVecView b_view(const_cast<DynVec &>(b));
        return on::lina::try_solve_dynamic<double>(a_view, b_view);
    }

    /**
     * @brief Add scalar to diagonal of matrix (for LM damping)
     *
     * @param m Matrix to modify (must be square)
     * @param value Value to add to each diagonal element
     */
    inline void add_diagonal(DynMat &m, double value) {
        const auto n = m.rows() < m.cols() ? m.rows() : m.cols();
        for (std::size_t i = 0; i < n; ++i) {
            m(i, i) += value;
        }
    }

    /**
     * @brief Compute Frobenius norm of a vector
     *
     * @param v Vector
     * @return L2 norm
     */
    inline double norm(const DynVec &v) {
        DynVecView view(const_cast<DynVec &>(v));
        return on::simd::norm(view);
    }

    /**
     * @brief Compute dot product of two vectors
     *
     * @param a First vector
     * @param b Second vector
     * @return Dot product
     */
    inline double dot(const DynVec &a, const DynVec &b) {
        DynVecView a_view(const_cast<DynVec &>(a));
        DynVecView b_view(const_cast<DynVec &>(b));
        return on::simd::dot(a_view, b_view);
    }

    /**
     * @brief Matrix-vector multiplication: result = A * x
     *
     * @param a Matrix (m x n)
     * @param x Vector (n elements)
     * @return Result vector (m elements)
     *
     * @note Uses manual loop for dynamic matrices. optinum's SIMD matmul_to()
     *       only supports fixed-size matrices. Consider adding runtime SIMD
     *       version to optinum for better performance with large dynamic matrices.
     */
    inline DynVec matmul(const DynMat &a, const DynVec &x) {
        const std::size_t m = a.rows();
        const std::size_t n = a.cols();

        DynVec result(m);

        // Column-major layout: accumulate column contributions
        for (std::size_t i = 0; i < m; ++i) {
            result[i] = 0.0;
        }
        for (std::size_t j = 0; j < n; ++j) {
            const double xj = x[j];
            for (std::size_t i = 0; i < m; ++i) {
                result[i] += a(i, j) * xj;
            }
        }

        return result;
    }

    /**
     * @brief Matrix-matrix multiplication: result = A * B
     *
     * @param a Matrix (m x k)
     * @param b Matrix (k x n)
     * @return Result matrix (m x n)
     *
     * @note Uses manual loop for dynamic matrices. optinum's SIMD matmul_to()
     *       only supports fixed-size matrices.
     */
    inline DynMat matmul(const DynMat &a, const DynMat &b) {
        const std::size_t m = a.rows();
        const std::size_t k = a.cols();
        const std::size_t n = b.cols();

        DynMat result(m, n);

        // Column-major GEMM: C(:,j) = A * B(:,j)
        for (std::size_t j = 0; j < n; ++j) {
            // Initialize column j of result to zero
            for (std::size_t i = 0; i < m; ++i) {
                result(i, j) = 0.0;
            }
            // Accumulate: result(:,j) += a(:,p) * b(p,j)
            for (std::size_t p = 0; p < k; ++p) {
                const double bpj = b(p, j);
                for (std::size_t i = 0; i < m; ++i) {
                    result(i, j) += a(i, p) * bpj;
                }
            }
        }

        return result;
    }

    /**
     * @brief Matrix transpose: result = A^T
     *
     * @param a Matrix (m x n)
     * @return Transposed matrix (n x m)
     *
     * @note Uses manual loop for dynamic matrices. optinum's SIMD transpose_to()
     *       only supports fixed-size matrices.
     */
    inline DynMat transpose(const DynMat &a) {
        const std::size_t m = a.rows();
        const std::size_t n = a.cols();

        DynMat result(n, m);

        // Transpose: result(j,i) = a(i,j)
        for (std::size_t j = 0; j < n; ++j) {
            for (std::size_t i = 0; i < m; ++i) {
                result(j, i) = a(i, j);
            }
        }

        return result;
    }

} // namespace graphix::factor
