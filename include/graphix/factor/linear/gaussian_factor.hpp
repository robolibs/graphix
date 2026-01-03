#pragma once

#include "graphix/factor/factor.hpp"
#include "graphix/kernel.hpp"

#include <datapod/matrix.hpp>
#include <datapod/sequential.hpp>

#include <map>

namespace graphix::factor {

    namespace dp = ::datapod;

    /**
     * @brief Gaussian factor - linearized version of a nonlinear factor
     *
     * Represents a linearized constraint of the form:
     *   error(x + dx) ≈ b + A * dx
     *
     * Where:
     * - b is the error vector at the linearization point
     * - A is the Jacobian matrix (derivative of error w.r.t. variables)
     * - dx is the delta from the linearization point
     *
     * For optimization, we want to minimize:
     *   0.5 * ||A * dx + b||²
     *
     * This is used in Gauss-Newton and Levenberg-Marquardt optimizers.
     */
    class GaussianFactor : public Factor {
      public:
        using Matrix = dp::mat::MatrixXd;
        using Vector = dp::mat::VectorXd;

        /**
         * @brief Construct a Gaussian factor
         *
         * @param keys Variables involved in this factor
         * @param jacobians Jacobian matrix for each variable (in order of keys)
         * @param b Error vector at linearization point
         *
         * For a factor with n keys and m-dimensional error:
         * - jacobians[i] is m × dim(key[i]) matrix
         * - b is m-dimensional vector
         *
         * Example: SE2BetweenFactor(pose_i, pose_j) with 3D error:
         * - keys = {pose_i, pose_j}
         * - jacobians[0] = 3×3 matrix (∂error/∂pose_i)
         * - jacobians[1] = 3×3 matrix (∂error/∂pose_j)
         * - b = 3D error vector
         */
        inline GaussianFactor(const dp::Vector<Key> &keys, const dp::Vector<Matrix> &jacobians, const Vector &b)
            : Factor(), jacobians_(jacobians), b_(b) {
            // Copy keys to parent
            for (std::size_t i = 0; i < keys.size(); ++i) {
                m_keys.push_back(keys[i]);
            }

            if (keys.size() != jacobians.size()) {
                throw std::invalid_argument("Number of keys must match number of Jacobians");
            }

            // Build key index map
            for (std::size_t i = 0; i < keys.size(); ++i) {
                key_index_[keys[i]] = i;
            }

            // Validate dimensions
            if (!jacobians.empty() && b.size() > 0) {
                std::size_t error_dim = b.size();
                for (std::size_t i = 0; i < jacobians.size(); ++i) {
                    if (jacobians[i].rows() != error_dim) {
                        throw std::invalid_argument("All Jacobians must have same number of rows as error vector");
                    }
                }
            }
        }

        /**
         * @brief Get Jacobian for a specific variable
         *
         * @param key Variable key
         * @return Jacobian matrix for that variable
         * @throws std::runtime_error if key not found
         */
        inline const Matrix &jacobian(Key key) const {
            auto it = key_index_.find(key);
            if (it == key_index_.end()) {
                throw std::runtime_error("Key not found in Gaussian factor");
            }
            return jacobians_[it->second];
        }

        /**
         * @brief Get all Jacobians in order of keys
         */
        const dp::Vector<Matrix> &jacobians() const { return jacobians_; }

        /**
         * @brief Get error vector at linearization point
         */
        const Vector &b() const { return b_; }

        /**
         * @brief Get dimension of error (size of error vector)
         */
        std::size_t dim() const { return b_.size(); }

        /**
         * @brief Compute weighted squared error for given delta
         *
         * Returns 0.5 * ||A * dx + b||²
         *
         * @param deltas Map from Key to delta vector for each variable
         * @return Weighted squared error
         */
        inline double error(const std::map<Key, Vector> &deltas) const {
            // Compute: 0.5 * ||A * dx + b||²

            // Start with b (copy)
            Vector residual(b_.size());
            for (std::size_t i = 0; i < b_.size(); ++i) {
                residual[i] = b_[i];
            }

            // Add A * dx for each variable
            for (std::size_t i = 0; i < m_keys.size(); ++i) {
                Key key = m_keys[i];
                const Matrix &J = jacobians_[i];

                // Get delta for this key (or zero if not in map)
                auto it = deltas.find(key);
                if (it != deltas.end()) {
                    const Vector &dx = it->second;

                    // Validate dimensions
                    if (dx.size() != J.cols()) {
                        throw std::invalid_argument("Delta dimension mismatch for key");
                    }

                    // Add J * dx to residual (manual matrix-vector multiply)
                    for (std::size_t row = 0; row < J.rows(); ++row) {
                        for (std::size_t col = 0; col < J.cols(); ++col) {
                            residual[row] += J(row, col) * dx[col];
                        }
                    }
                }
                // If delta not provided, assume zero (no contribution)
            }

            // Compute 0.5 * ||residual||²
            double sum_squared = 0.0;
            for (std::size_t i = 0; i < residual.size(); ++i) {
                sum_squared += residual[i] * residual[i];
            }

            return 0.5 * sum_squared;
        }

      private:
        dp::Vector<Matrix> jacobians_;         ///< Jacobian for each variable (in order of keys)
        Vector b_;                             ///< Error vector at linearization point
        std::map<Key, std::size_t> key_index_; ///< Map from Key to index in jacobians_
    };

} // namespace graphix::factor
