#pragma once

#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/types/vec3d.hpp"
#include <stdexcept>

namespace graphix::factor {

    /**
     * @brief Prior factor for Vec3d variables
     *
     * Constrains a Vec3d variable to be near a prior value.
     * Error = 0.5 * sum((x[i] - prior[i]) / sigma[i])^2
     *
     * Example uses:
     * - GPS measurement: Vec3d(x, y, theta) with known position
     * - Initial pose constraint
     * - Anchor point in optimization
     */
    class Vec3PriorFactor : public NonlinearFactor {
      public:
        /**
         * @brief Construct prior factor
         *
         * @param key Variable key
         * @param prior Prior value (mean)
         * @param sigmas Standard deviations for each dimension
         */
        Vec3PriorFactor(Key key, const Vec3d &prior, const Vec3d &sigmas);

        /**
         * @brief Compute error for this factor
         *
         * Error = 0.5 * sum((x[i] - prior[i]) / sigma[i])^2
         *
         * @param values Current variable values
         * @return Squared Mahalanobis distance (weighted squared error)
         */
        double error(const Values &values) const override;

        /**
         * @brief Get prior value
         */
        const Vec3d &prior() const { return prior_; }

        /**
         * @brief Get sigmas
         */
        const Vec3d &sigmas() const { return sigmas_; }

      private:
        Vec3d prior_;  ///< Prior value (mean)
        Vec3d sigmas_; ///< Standard deviations (diagonal covariance)
    };

} // namespace graphix::factor
