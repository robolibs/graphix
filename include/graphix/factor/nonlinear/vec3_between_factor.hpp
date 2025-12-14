#pragma once

#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/types/vec3d.hpp"
#include <stdexcept>

namespace graphix::factor {

    /**
     * @brief Between factor for Vec3d variables
     *
     * Constrains the relative difference between two Vec3d variables.
     *
     * When interpreting Vec3d as a 2D pose (x, y, theta), the measured (dx, dy, dtheta)
     * is treated as an odometry measurement in the *local frame of i*:
     *   predicted_translation_local = R(-theta_i) * (t_j - t_i)
     *   predicted_rotation = wrap(theta_j - theta_i)
     *
     * Error = 0.5 * sum(((predicted - measured) / sigma[i])^2
     *
     * Example uses:
     * - Odometry: Vec3d(dx, dy, dtheta) between poses
     * - Relative measurements
     * - Loop closures
     */
    class Vec3BetweenFactor : public NonlinearFactor {
      public:
        /**
         * @brief Construct between factor
         *
         * @param key_i First variable key
         * @param key_j Second variable key
         * @param measured Measured relative difference (vj - vi)
         * @param sigmas Standard deviations for each dimension
         */
        Vec3BetweenFactor(Key key_i, Key key_j, const Vec3d &measured, const Vec3d &sigmas);

        /**
         * @brief Compute error for this factor
         *
         * Error = 0.5 * sum(((predicted - measured)[i] / sigma[i])^2)
         *
         * @param values Current variable values
         * @return Squared Mahalanobis distance (weighted squared error)
         */
        double error(const Values &values) const override;

        /**
         * @brief Get measured value
         */
        const Vec3d &measured() const { return measured_; }

        /**
         * @brief Get sigmas
         */
        const Vec3d &sigmas() const { return sigmas_; }

      private:
        Vec3d measured_; ///< Measured relative difference
        Vec3d sigmas_;   ///< Standard deviations (diagonal covariance)
    };

} // namespace graphix::factor
