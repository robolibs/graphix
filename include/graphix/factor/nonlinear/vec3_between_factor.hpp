#pragma once

#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include <cmath>
#include <datapod/matrix.hpp>
#include <stdexcept>

namespace graphix::factor {

    namespace detail {
        inline double wrap_angle_pi(double angle_rad) {
            constexpr double kPi = 3.14159265358979323846;
            constexpr double kTwoPi = 2.0 * kPi;
            while (angle_rad > kPi) {
                angle_rad -= kTwoPi;
            }
            while (angle_rad < -kPi) {
                angle_rad += kTwoPi;
            }
            return angle_rad;
        }
    } // namespace detail

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
        using Vec3d = datapod::mat::vector3d;

        /**
         * @brief Construct between factor
         *
         * @param key_i First variable key
         * @param key_j Second variable key
         * @param measured Measured relative difference (vj - vi)
         * @param sigmas Standard deviations for each dimension
         */
        inline Vec3BetweenFactor(Key key_i, Key key_j, const Vec3d &measured, const Vec3d &sigmas)
            : NonlinearFactor({key_i, key_j}), measured_(measured), sigmas_(sigmas) {

            // Validate sigmas
            if (sigmas_[0] <= 0.0 || sigmas_[1] <= 0.0 || sigmas_[2] <= 0.0) {
                throw std::invalid_argument("All sigmas must be positive");
            }
        }

        /**
         * @brief Compute error for this factor
         *
         * Error = 0.5 * sum(((predicted - measured)[i] / sigma[i])^2)
         *
         * @param values Current variable values
         * @return Squared Mahalanobis distance (weighted squared error)
         */
        inline double error(const Values &values) const override {
            // Get the two variable values
            Vec3d vi = values.at<Vec3d>(keys()[0]);
            Vec3d vj = values.at<Vec3d>(keys()[1]);

            // Interpret Vec3d as 2D pose (x, y, theta).
            // Measurement is in the local frame of pose i (odometry-style).
            const double theta_i = vi[2];
            const double dx_world = vj[0] - vi[0];
            const double dy_world = vj[1] - vi[1];

            // Rotate world delta into i frame: R(-theta_i) * (t_j - t_i)
            const double c = std::cos(theta_i);
            const double s = std::sin(theta_i);
            const double dx_local = c * dx_world + s * dy_world;
            const double dy_local = -s * dx_world + c * dy_world;

            const double dtheta = detail::wrap_angle_pi(vj[2] - vi[2]);

            Vec3d predicted{dx_local, dy_local, dtheta};

            // Residual (wrap angle component)
            Vec3d diff{predicted[0] - measured_[0], predicted[1] - measured_[1],
                       detail::wrap_angle_pi(predicted[2] - measured_[2])};

            // Compute weighted squared error: sum((diff[i] / sigma[i])^2)
            double squared_error = 0.0;
            for (int i = 0; i < 3; i++) {
                double weighted = diff[i] / sigmas_[i];
                squared_error += weighted * weighted;
            }

            // Apply robust loss function if present
            if (loss_function_) {
                return loss_function_->evaluate(squared_error);
            }

            return 0.5 * squared_error;
        }

        /**
         * @brief Get dimension of variable (always 3 for Vec3d)
         */
        size_t dim(Key) const override { return 3; }

        /**
         * @brief Compute error vector for linearization
         *
         * Returns the weighted residual: (predicted - measured) ./ sigma
         */
        inline datapod::mat::VectorXd error_vector(const Values &values) const override {
            // Get the two variable values
            Vec3d vi = values.at<Vec3d>(keys()[0]);
            Vec3d vj = values.at<Vec3d>(keys()[1]);

            // Interpret Vec3d as 2D pose (x, y, theta).
            // Measurement is in the local frame of pose i (odometry-style).
            const double theta_i = vi[2];
            const double dx_world = vj[0] - vi[0];
            const double dy_world = vj[1] - vi[1];

            // Rotate world delta into i frame: R(-theta_i) * (t_j - t_i)
            const double c = std::cos(theta_i);
            const double s = std::sin(theta_i);
            const double dx_local = c * dx_world + s * dy_world;
            const double dy_local = -s * dx_world + c * dy_world;

            const double dtheta = detail::wrap_angle_pi(vj[2] - vi[2]);

            Vec3d predicted{dx_local, dy_local, dtheta};

            // Residual (wrap angle component)
            Vec3d diff{predicted[0] - measured_[0], predicted[1] - measured_[1],
                       detail::wrap_angle_pi(predicted[2] - measured_[2])};

            // Return weighted residual
            return datapod::mat::VectorXd{diff[0] / sigmas_[0], diff[1] / sigmas_[1], diff[2] / sigmas_[2]};
        }

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
