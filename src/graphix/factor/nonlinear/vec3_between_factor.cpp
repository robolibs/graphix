#include "graphix/factor/nonlinear/vec3_between_factor.hpp"

namespace graphix::factor {

    Vec3BetweenFactor::Vec3BetweenFactor(Key key_i, Key key_j, const Vec3d &measured, const Vec3d &sigmas)
        : NonlinearFactor({key_i, key_j}), measured_(measured), sigmas_(sigmas) {

        // Validate sigmas
        if (sigmas.x() <= 0.0 || sigmas.y() <= 0.0 || sigmas.z() <= 0.0) {
            throw std::invalid_argument("All sigmas must be positive");
        }
    }

    double Vec3BetweenFactor::error(const Values &values) const {
        // Get the two variable values
        Vec3d vi = values.at<Vec3d>(keys()[0]);
        Vec3d vj = values.at<Vec3d>(keys()[1]);

        // Compute relative difference
        Vec3d relative = vj - vi;

        // Compute difference from measurement
        Vec3d diff = relative - measured_;

        // Compute weighted squared error: 0.5 * sum((diff[i] / sigma[i])^2)
        double error = 0.0;
        for (int i = 0; i < 3; i++) {
            double weighted = diff[i] / sigmas_[i];
            error += weighted * weighted;
        }

        return 0.5 * error;
    }

} // namespace graphix::factor
