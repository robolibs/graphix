#include "doctest.h"
#include "graphix/factor/types/vec3d.hpp"
#include <cmath>

using namespace graphix::factor;

TEST_CASE("Vec3d - default constructor") {
    Vec3d v;
    CHECK(v.x() == 0.0);
    CHECK(v.y() == 0.0);
    CHECK(v.z() == 0.0);
}

TEST_CASE("Vec3d - constructor with values") {
    Vec3d v(1.0, 2.0, 3.0);
    CHECK(v.x() == 1.0);
    CHECK(v.y() == 2.0);
    CHECK(v.z() == 3.0);
}

TEST_CASE("Vec3d - constructor from array") {
    double arr[3] = {4.0, 5.0, 6.0};
    Vec3d v(arr);
    CHECK(v.x() == 4.0);
    CHECK(v.y() == 5.0);
    CHECK(v.z() == 6.0);
}

TEST_CASE("Vec3d - named accessors") {
    Vec3d v(1.5, 2.5, 3.5);
    CHECK(v.x() == doctest::Approx(1.5));
    CHECK(v.y() == doctest::Approx(2.5));
    CHECK(v.z() == doctest::Approx(3.5));
}

TEST_CASE("Vec3d - index accessors (const)") {
    Vec3d v(10.0, 20.0, 30.0);
    CHECK(v[0] == 10.0);
    CHECK(v[1] == 20.0);
    CHECK(v[2] == 30.0);
}

TEST_CASE("Vec3d - index accessors (mutable)") {
    Vec3d v;
    v[0] = 7.0;
    v[1] = 8.0;
    v[2] = 9.0;
    CHECK(v.x() == 7.0);
    CHECK(v.y() == 8.0);
    CHECK(v.z() == 9.0);
}

TEST_CASE("Vec3d - mutable named accessors") {
    Vec3d v;
    v.x() = 11.0;
    v.y() = 12.0;
    v.z() = 13.0;
    CHECK(v[0] == 11.0);
    CHECK(v[1] == 12.0);
    CHECK(v[2] == 13.0);
}

TEST_CASE("Vec3d - addition") {
    Vec3d v1(1.0, 2.0, 3.0);
    Vec3d v2(4.0, 5.0, 6.0);
    Vec3d result = v1 + v2;
    CHECK(result.x() == 5.0);
    CHECK(result.y() == 7.0);
    CHECK(result.z() == 9.0);
}

TEST_CASE("Vec3d - subtraction") {
    Vec3d v1(10.0, 20.0, 30.0);
    Vec3d v2(1.0, 2.0, 3.0);
    Vec3d result = v1 - v2;
    CHECK(result.x() == 9.0);
    CHECK(result.y() == 18.0);
    CHECK(result.z() == 27.0);
}

TEST_CASE("Vec3d - scalar multiplication") {
    Vec3d v(1.0, 2.0, 3.0);
    Vec3d result = v * 2.0;
    CHECK(result.x() == 2.0);
    CHECK(result.y() == 4.0);
    CHECK(result.z() == 6.0);
}

TEST_CASE("Vec3d - scalar multiplication (reverse)") {
    Vec3d v(1.0, 2.0, 3.0);
    Vec3d result = 3.0 * v;
    CHECK(result.x() == 3.0);
    CHECK(result.y() == 6.0);
    CHECK(result.z() == 9.0);
}

TEST_CASE("Vec3d - scalar division") {
    Vec3d v(10.0, 20.0, 30.0);
    Vec3d result = v / 2.0;
    CHECK(result.x() == 5.0);
    CHECK(result.y() == 10.0);
    CHECK(result.z() == 15.0);
}

TEST_CASE("Vec3d - in-place addition") {
    Vec3d v(1.0, 2.0, 3.0);
    v += Vec3d(1.0, 1.0, 1.0);
    CHECK(v.x() == 2.0);
    CHECK(v.y() == 3.0);
    CHECK(v.z() == 4.0);
}

TEST_CASE("Vec3d - in-place subtraction") {
    Vec3d v(10.0, 20.0, 30.0);
    v -= Vec3d(1.0, 2.0, 3.0);
    CHECK(v.x() == 9.0);
    CHECK(v.y() == 18.0);
    CHECK(v.z() == 27.0);
}

TEST_CASE("Vec3d - in-place multiplication") {
    Vec3d v(2.0, 3.0, 4.0);
    v *= 2.0;
    CHECK(v.x() == 4.0);
    CHECK(v.y() == 6.0);
    CHECK(v.z() == 8.0);
}

TEST_CASE("Vec3d - in-place division") {
    Vec3d v(20.0, 30.0, 40.0);
    v /= 2.0;
    CHECK(v.x() == 10.0);
    CHECK(v.y() == 15.0);
    CHECK(v.z() == 20.0);
}

TEST_CASE("Vec3d - unary minus") {
    Vec3d v(1.0, -2.0, 3.0);
    Vec3d result = -v;
    CHECK(result.x() == -1.0);
    CHECK(result.y() == 2.0);
    CHECK(result.z() == -3.0);
}

TEST_CASE("Vec3d - norm of unit vector") {
    Vec3d v(1.0, 0.0, 0.0);
    CHECK(v.norm() == doctest::Approx(1.0));
}

TEST_CASE("Vec3d - norm of 3-4-5 triangle") {
    Vec3d v(0.0, 3.0, 4.0);
    CHECK(v.norm() == doctest::Approx(5.0));
}

TEST_CASE("Vec3d - norm of general vector") {
    Vec3d v(1.0, 2.0, 2.0);
    // sqrt(1 + 4 + 4) = sqrt(9) = 3
    CHECK(v.norm() == doctest::Approx(3.0));
}

TEST_CASE("Vec3d - norm_squared") {
    Vec3d v(1.0, 2.0, 2.0);
    CHECK(v.norm_squared() == doctest::Approx(9.0));
}

TEST_CASE("Vec3d - norm of zero vector") {
    Vec3d v(0.0, 0.0, 0.0);
    CHECK(v.norm() == doctest::Approx(0.0));
}

TEST_CASE("Vec3d - dot product orthogonal") {
    Vec3d v1(1.0, 0.0, 0.0);
    Vec3d v2(0.0, 1.0, 0.0);
    CHECK(v1.dot(v2) == doctest::Approx(0.0));
}

TEST_CASE("Vec3d - dot product parallel") {
    Vec3d v1(2.0, 0.0, 0.0);
    Vec3d v2(3.0, 0.0, 0.0);
    CHECK(v1.dot(v2) == doctest::Approx(6.0));
}

TEST_CASE("Vec3d - dot product general") {
    Vec3d v1(1.0, 2.0, 3.0);
    Vec3d v2(4.0, 5.0, 6.0);
    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    CHECK(v1.dot(v2) == doctest::Approx(32.0));
}

TEST_CASE("Vec3d - normalized unit vector") {
    Vec3d v(5.0, 0.0, 0.0);
    Vec3d n = v.normalized();
    CHECK(n.x() == doctest::Approx(1.0));
    CHECK(n.y() == doctest::Approx(0.0));
    CHECK(n.z() == doctest::Approx(0.0));
    CHECK(n.norm() == doctest::Approx(1.0));
}

TEST_CASE("Vec3d - normalized general vector") {
    Vec3d v(3.0, 4.0, 0.0);
    Vec3d n = v.normalized();
    CHECK(n.x() == doctest::Approx(0.6));
    CHECK(n.y() == doctest::Approx(0.8));
    CHECK(n.z() == doctest::Approx(0.0));
    CHECK(n.norm() == doctest::Approx(1.0));
}

TEST_CASE("Vec3d - normalized zero vector") {
    Vec3d v(0.0, 0.0, 0.0);
    Vec3d n = v.normalized();
    CHECK(n.x() == doctest::Approx(0.0));
    CHECK(n.y() == doctest::Approx(0.0));
    CHECK(n.z() == doctest::Approx(0.0));
}

TEST_CASE("Vec3d - data pointer access") {
    Vec3d v(1.0, 2.0, 3.0);
    const double *ptr = v.data();
    CHECK(ptr[0] == 1.0);
    CHECK(ptr[1] == 2.0);
    CHECK(ptr[2] == 3.0);
}

TEST_CASE("Vec3d - mutable data pointer") {
    Vec3d v;
    double *ptr = v.data();
    ptr[0] = 7.0;
    ptr[1] = 8.0;
    ptr[2] = 9.0;
    CHECK(v.x() == 7.0);
    CHECK(v.y() == 8.0);
    CHECK(v.z() == 9.0);
}

TEST_CASE("Vec3d - equality operator") {
    Vec3d v1(1.0, 2.0, 3.0);
    Vec3d v2(1.0, 2.0, 3.0);
    CHECK(v1 == v2);
}

TEST_CASE("Vec3d - inequality operator") {
    Vec3d v1(1.0, 2.0, 3.0);
    Vec3d v2(1.0, 2.0, 3.1);
    CHECK(v1 != v2);
}

TEST_CASE("Vec3d - negative values") {
    Vec3d v(-1.0, -2.0, -3.0);
    CHECK(v.x() == -1.0);
    CHECK(v.y() == -2.0);
    CHECK(v.z() == -3.0);
}

TEST_CASE("Vec3d - use as 2D pose (x, y, theta)") {
    // Semantic interpretation: 2D pose at (1, 2) with orientation π/4
    Vec3d pose(1.0, 2.0, M_PI / 4);
    CHECK(pose.x() == doctest::Approx(1.0));
    CHECK(pose.y() == doctest::Approx(2.0));
    CHECK(pose.z() == doctest::Approx(M_PI / 4));
}

TEST_CASE("Vec3d - use as velocity (vx, vy, omega)") {
    // Semantic interpretation: velocity 1 m/s forward, 0.5 m/s sideways, 0.1 rad/s rotation
    Vec3d vel(1.0, 0.5, 0.1);
    CHECK(vel[0] == doctest::Approx(1.0));
    CHECK(vel[1] == doctest::Approx(0.5));
    CHECK(vel[2] == doctest::Approx(0.1));
}

TEST_CASE("Vec3d - chained operations") {
    Vec3d v1(1.0, 2.0, 3.0);
    Vec3d v2(2.0, 3.0, 4.0);
    Vec3d result = (v1 + v2) * 2.0 - Vec3d(1.0, 1.0, 1.0);
    // (1+2)*2 - 1 = 6 - 1 = 5
    // (2+3)*2 - 1 = 10 - 1 = 9
    // (3+4)*2 - 1 = 14 - 1 = 13
    CHECK(result.x() == doctest::Approx(5.0));
    CHECK(result.y() == doctest::Approx(9.0));
    CHECK(result.z() == doctest::Approx(13.0));
}
