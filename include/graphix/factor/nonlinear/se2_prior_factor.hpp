#pragma once

/**
 * @file se2_prior_factor.hpp
 * @brief Prior factor for SE2 poses using optinum Lie groups
 *
 * This factor constrains an SE2 pose to a prior value using proper
 * manifold operations (exp/log maps) rather than Euclidean differences.
 */

#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/types.hpp"

#include <cmath>
#include <stdexcept>

namespace graphix::factor {

    /**
     * @brief Prior factor for SE2 poses
     *
     * Constrains an SE2 pose to match a prior value. The error is computed
     * in the tangent space using the logarithmic map:
     *
     *   error = log(prior^{-1} * pose)
     *
     * This gives a proper manifold error that respects the SE2 geometry.
     *
     * The weighted squared error is:
     *   E = 0.5 * sum((error[i] / sigma[i])^2)
     *
     * where error = [dx, dy, dtheta] in the tangent space.
     */
    class SE2PriorFactor : public NonlinearFactor {
      public:
        using Tangent = SE2d::Tangent; // dp::mat::vector<double, 3>

        /**
         * @brief Construct SE2 prior factor
         *
         * @param key Variable key for the pose
         * @param prior Prior SE2 pose value
         * @param sigmas Standard deviations [sigma_x, sigma_y, sigma_theta]
         */
        SE2PriorFactor(Key key, const SE2d &prior, const Vec3d &sigmas)
            : NonlinearFactor({key}), prior_(prior), sigmas_(sigmas) {

            if (sigmas_[0] <= 0.0 || sigmas_[1] <= 0.0 || sigmas_[2] <= 0.0) {
                throw std::invalid_argument("All sigmas must be positive");
            }
        }

        /**
         * @brief Compute error for this factor
         *
         * @param values Current variable values
         * @return Weighted squared error (scalar)
         */
        double error(const Values &values) const override {
            SE2d pose = values.at<SE2d>(keys()[0]);

            // Compute error in tangent space: log(prior^{-1} * pose)
            Tangent delta = (prior_.inverse() * pose).log();

            // Weighted squared error
            double squared_error = 0.0;
            for (int i = 0; i < 3; ++i) {
                double weighted = delta[i] / sigmas_[i];
                squared_error += weighted * weighted;
            }

            // Apply robust loss function if present
            if (loss_function_) {
                return loss_function_->evaluate(squared_error);
            }

            return 0.5 * squared_error;
        }

        /**
         * @brief Get dimension of variable (always 3 for SE2 tangent space)
         */
        std::size_t dim(Key) const override { return 3; }

        /**
         * @brief Compute error vector for linearization
         *
         * Returns the weighted residual in tangent space.
         */
        dp::mat::VectorXd error_vector(const Values &values) const override {
            SE2d pose = values.at<SE2d>(keys()[0]);

            // Compute error in tangent space
            Tangent delta = (prior_.inverse() * pose).log();

            // Return weighted residual
            dp::mat::VectorXd result(3);
            result[0] = delta[0] / sigmas_[0];
            result[1] = delta[1] / sigmas_[1];
            result[2] = delta[2] / sigmas_[2];
            return result;
        }

        /**
         * @brief Get prior pose
         */
        const SE2d &prior() const { return prior_; }

        /**
         * @brief Get sigmas
         */
        const Vec3d &sigmas() const { return sigmas_; }

      private:
        SE2d prior_;   ///< Prior pose value
        Vec3d sigmas_; ///< Standard deviations [sigma_x, sigma_y, sigma_theta]
    };

} // namespace graphix::factor
