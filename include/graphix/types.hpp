#pragma once

/**
 * @file types.hpp
 * @brief Central type definitions for the graphix library
 *
 * Architecture:
 * - DATAPOD owns all data (dp::mat::vector, dp::mat::matrix, dp::Vector, etc.)
 * - OPTINUM provides SIMD operations on datapod types (does NOT own data)
 *
 * Usage:
 * - Use these type aliases for storage/member variables
 * - Use optinum functions (on::simd::*, on::lina::*) for operations
 *
 * IMPORTANT: datapod matrices use COLUMN-MAJOR storage layout.
 */

#include <datapod/associative.hpp>
#include <datapod/matrix.hpp>
#include <datapod/sequential.hpp>
#include <datapod/trees.hpp>

#include <optinum/lina/lina.hpp>
#include <optinum/simd/simd.hpp>

namespace graphix {

    namespace dp = ::datapod;
    namespace on = ::optinum;

    // =========================================================================
    // Fixed-size vector types (data storage from datapod::mat)
    // =========================================================================

    using Vec3d = dp::mat::vector3d;
    using Vec3f = dp::mat::vector3f;
    using Vec4d = dp::mat::vector4d;
    using Vec4f = dp::mat::vector4f;
    using Vec6d = dp::mat::vector6d;
    using Vec6f = dp::mat::vector6f;

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
    using Mat4d = dp::mat::matrix4x4d;
    using Mat4f = dp::mat::matrix4x4f;
    using Mat6d = dp::mat::matrix6x6d;
    using Mat6f = dp::mat::matrix6x6f;

    // =========================================================================
    // Container types (from datapod)
    // =========================================================================

    template <typename T> using Vector = dp::Vector<T>;

    template <typename K, typename V> using Map = dp::Map<K, V>;

    template <typename T> using Set = dp::Set<T>;

    template <typename K, typename V> using OrderedMap = dp::OrderedMap<K, V>;

    template <typename T> using OrderedSet = dp::OrderedSet<T>;

    // =========================================================================
    // Helper functions for matrix operations
    // =========================================================================

    inline void add_diagonal(DynMat &m, double value) {
        const auto n = m.rows() < m.cols() ? m.rows() : m.cols();
        for (std::size_t i = 0; i < n; ++i) {
            m(i, i) += value;
        }
    }

    inline void add_diagonal(DynMatF &m, float value) {
        const auto n = m.rows() < m.cols() ? m.rows() : m.cols();
        for (std::size_t i = 0; i < n; ++i) {
            m(i, i) += value;
        }
    }

} // namespace graphix
