#include "graphix/factor/nonlinear/vec3_between_factor.hpp"

#include <cmath>

namespace graphix::factor {

    static double wrap_angle_pi(double angle_rad) {
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

        // Interpret Vec3d as 2D pose (x, y, theta).
        // Measurement is in the local frame of pose i (odometry-style).
        const double theta_i = vi.z();
        const double dx_world = vj.x() - vi.x();
        const double dy_world = vj.y() - vi.y();

        // Rotate world delta into i frame: R(-theta_i) * (t_j - t_i)
        const double c = std::cos(theta_i);
        const double s = std::sin(theta_i);
        const double dx_local = c * dx_world + s * dy_world;
        const double dy_local = -s * dx_world + c * dy_world;

        const double dtheta = wrap_angle_pi(vj.z() - vi.z());

        Vec3d predicted(dx_local, dy_local, dtheta);

        // Residual (wrap angle component)
        Vec3d diff = predicted - measured_;
        diff.z() = wrap_angle_pi(diff.z());

        // Compute weighted squared error: 0.5 * sum((diff[i] / sigma[i])^2)
        double error = 0.0;
        for (int i = 0; i < 3; i++) {
            double weighted = diff[i] / sigmas_[i];
            error += weighted * weighted;
        }

        return 0.5 * error;
    }

    std::vector<double> Vec3BetweenFactor::error_vector(const Values &values) const {
        // Get the two variable values
        Vec3d vi = values.at<Vec3d>(keys()[0]);
        Vec3d vj = values.at<Vec3d>(keys()[1]);

        // Interpret Vec3d as 2D pose (x, y, theta).
        // Measurement is in the local frame of pose i (odometry-style).
        const double theta_i = vi.z();
        const double dx_world = vj.x() - vi.x();
        const double dy_world = vj.y() - vi.y();

        // Rotate world delta into i frame: R(-theta_i) * (t_j - t_i)
        const double c = std::cos(theta_i);
        const double s = std::sin(theta_i);
        const double dx_local = c * dx_world + s * dy_world;
        const double dy_local = -s * dx_world + c * dy_world;

        const double dtheta = wrap_angle_pi(vj.z() - vi.z());

        Vec3d predicted(dx_local, dy_local, dtheta);

        // Residual (wrap angle component)
        Vec3d diff = predicted - measured_;
        diff.z() = wrap_angle_pi(diff.z());

        // Return weighted residual
        return {diff.x() / sigmas_.x(), diff.y() / sigmas_.y(), diff.z() / sigmas_.z()};
    }

} // namespace graphix::factor
