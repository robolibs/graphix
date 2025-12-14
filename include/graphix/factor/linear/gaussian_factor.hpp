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
        GaussianFactor(const std::vector<Key> &keys, const std::vector<Matrix> &jacobians,
                       const std::vector<double> &b);

        /**
         * @brief Get Jacobian for a specific variable
         *
         * @param key Variable key
         * @return Jacobian matrix for that variable
         * @throws std::runtime_error if key not found
         */
        const Matrix &jacobian(Key key) const;

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
        double error(const std::map<Key, std::vector<double>> &deltas) const;

      private:
        std::vector<Matrix> jacobians_;   ///< Jacobian for each variable (in order of keys)
        std::vector<double> b_;           ///< Error vector at linearization point
        std::map<Key, size_t> key_index_; ///< Map from Key to index in jacobians_
    };

} // namespace graphix::factor
