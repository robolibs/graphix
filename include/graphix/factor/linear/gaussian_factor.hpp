#pragma once

#include "graphix/factor/factor.hpp"
#include "graphix/factor/linear/matrix.hpp"
#include "graphix/kernel.hpp"
#include <map>
#include <vector>

namespace graphix::factor {

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
         * Example: Vec3BetweenFactor(pose_i, pose_j) with 3D error:
         * - keys = {pose_i, pose_j}
         * - jacobians[0] = 3×3 matrix (∂error/∂pose_i)
         * - jacobians[1] = 3×3 matrix (∂error/∂pose_j)
         * - b = 3D error vector
         */
        inline GaussianFactor(const std::vector<Key> &keys, const std::vector<Matrix> &jacobians,
                              const std::vector<double> &b)
            : Factor(), jacobians_(jacobians), b_(b) {
            // Copy keys to parent
            for (Key key : keys) {
                m_keys.push_back(key);
            }

            if (keys.size() != jacobians.size()) {
                throw std::invalid_argument("Number of keys must match number of Jacobians");
            }

            // Build key index map
            for (size_t i = 0; i < keys.size(); ++i) {
                key_index_[keys[i]] = i;
            }

            // Validate dimensions
            if (!jacobians.empty() && !b.empty()) {
                size_t error_dim = b.size();
                for (size_t i = 0; i < jacobians.size(); ++i) {
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
        const std::vector<Matrix> &jacobians() const { return jacobians_; }

        /**
         * @brief Get error vector at linearization point
         */
        const std::vector<double> &b() const { return b_; }

        /**
         * @brief Get dimension of error (size of error vector)
         */
        size_t dim() const { return b_.size(); }

        /**
         * @brief Compute weighted squared error for given delta
         *
         * Returns 0.5 * ||A * dx + b||²
         *
         * @param deltas Map from Key to delta vector for each variable
         * @return Weighted squared error
         */
        inline double error(const std::map<Key, std::vector<double>> &deltas) const {
            // Compute: 0.5 * ||A * dx + b||²

            // Start with b
            std::vector<double> residual = b_;

            // Add A * dx for each variable
            for (size_t i = 0; i < m_keys.size(); ++i) {
                Key key = m_keys[i];

                // Get delta for this key (or zero if not in map)
                std::vector<double> dx;
                auto it = deltas.find(key);
                if (it != deltas.end()) {
                    dx = it->second;
                } else {
                    // Delta not provided, assume zero
                    dx.resize(jacobians_[i].cols(), 0.0);
                }

                // Validate dimensions
                if (dx.size() != jacobians_[i].cols()) {
                    throw std::invalid_argument("Delta dimension mismatch for key");
                }

                // Add J_i * dx_i to residual
                std::vector<double> J_dx = jacobians_[i] * dx;
                for (size_t j = 0; j < residual.size(); ++j) {
                    residual[j] += J_dx[j];
                }
            }

            // Compute 0.5 * ||residual||²
            double sum_squared = 0.0;
            for (double r : residual) {
                sum_squared += r * r;
            }

            return 0.5 * sum_squared;
        }

      private:
        std::vector<Matrix> jacobians_;   ///< Jacobian for each variable (in order of keys)
        std::vector<double> b_;           ///< Error vector at linearization point
        std::map<Key, size_t> key_index_; ///< Map from Key to index in jacobians_
    };

} // namespace graphix::factor
