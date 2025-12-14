#pragma once

#include <cmath>
#include <iostream>

namespace graphix::factor {

    /**
     * @brief Generic 3-dimensional vector
     *
     * Can represent:
     * - 2D poses: Vec3d(x, y, theta)
     * - 2D velocities: Vec3d(vx, vy, omega)
     * - 3D positions: Vec3d(x, y, z)
     * - Any 3D data
     *
     * Semantic meaning is defined by how you use it, not the type itself.
     */
    class Vec3d {
      public:
        /**
         * @brief Default constructor - initializes to zero
         */
        Vec3d() : data_{0.0, 0.0, 0.0} {}

        /**
         * @brief Construct from three values
         */
        Vec3d(double x, double y, double z) : data_{x, y, z} {}

        /**
         * @brief Construct from array
         */
        explicit Vec3d(const double *arr) : data_{arr[0], arr[1], arr[2]} {}

        // Accessors by name
        double x() const { return data_[0]; }
        double y() const { return data_[1]; }
        double z() const { return data_[2]; }

        double &x() { return data_[0]; }
        double &y() { return data_[1]; }
        double &z() { return data_[2]; }

        // Accessors by index
        double operator[](size_t i) const { return data_[i]; }
        double &operator[](size_t i) { return data_[i]; }

        // Vector arithmetic
        Vec3d operator+(const Vec3d &other) const {
            return Vec3d(data_[0] + other.data_[0], data_[1] + other.data_[1], data_[2] + other.data_[2]);
        }

        Vec3d operator-(const Vec3d &other) const {
            return Vec3d(data_[0] - other.data_[0], data_[1] - other.data_[1], data_[2] - other.data_[2]);
        }

        Vec3d operator*(double scalar) const { return Vec3d(data_[0] * scalar, data_[1] * scalar, data_[2] * scalar); }

        Vec3d operator/(double scalar) const { return Vec3d(data_[0] / scalar, data_[1] / scalar, data_[2] / scalar); }

        // In-place operations
        Vec3d &operator+=(const Vec3d &other) {
            data_[0] += other.data_[0];
            data_[1] += other.data_[1];
            data_[2] += other.data_[2];
            return *this;
        }

        Vec3d &operator-=(const Vec3d &other) {
            data_[0] -= other.data_[0];
            data_[1] -= other.data_[1];
            data_[2] -= other.data_[2];
            return *this;
        }

        Vec3d &operator*=(double scalar) {
            data_[0] *= scalar;
            data_[1] *= scalar;
            data_[2] *= scalar;
            return *this;
        }

        Vec3d &operator/=(double scalar) {
            data_[0] /= scalar;
            data_[1] /= scalar;
            data_[2] /= scalar;
            return *this;
        }

        // Unary minus
        Vec3d operator-() const { return Vec3d(-data_[0], -data_[1], -data_[2]); }

        /**
         * @brief Euclidean norm (L2 norm)
         */
        double norm() const { return std::sqrt(data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2]); }

        /**
         * @brief Squared norm (avoids sqrt)
         */
        double norm_squared() const { return data_[0] * data_[0] + data_[1] * data_[1] + data_[2] * data_[2]; }

        /**
         * @brief Dot product
         */
        double dot(const Vec3d &other) const {
            return data_[0] * other.data_[0] + data_[1] * other.data_[1] + data_[2] * other.data_[2];
        }

        /**
         * @brief Normalize to unit length
         */
        Vec3d normalized() const {
            double n = norm();
            if (n < 1e-10) {
                return Vec3d(0, 0, 0);
            }
            return *this / n;
        }

        /**
         * @brief Raw data access
         */
        const double *data() const { return data_; }
        double *data() { return data_; }

        /**
         * @brief Comparison operators
         */
        bool operator==(const Vec3d &other) const {
            return data_[0] == other.data_[0] && data_[1] == other.data_[1] && data_[2] == other.data_[2];
        }

        bool operator!=(const Vec3d &other) const { return !(*this == other); }

      private:
        double data_[3];
    };

    /**
     * @brief Scalar * Vec3d
     */
    inline Vec3d operator*(double scalar, const Vec3d &vec) { return vec * scalar; }

    /**
     * @brief Print Vec3d
     */
    inline std::ostream &operator<<(std::ostream &os, const Vec3d &vec) {
        os << "Vec3d(" << vec.x() << ", " << vec.y() << ", " << vec.z() << ")";
        return os;
    }

} // namespace graphix::factor
