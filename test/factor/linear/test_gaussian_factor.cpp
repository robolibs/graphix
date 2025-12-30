#include "graphix/factor/linear/gaussian_factor.hpp"
#include "graphix/factor/nonlinear/se2_between_factor.hpp"
#include "graphix/factor/nonlinear/se2_prior_factor.hpp"
#include "graphix/factor/types.hpp"
#include <datapod/sequential.hpp>
#include <doctest/doctest.h>

using namespace graphix;
using namespace graphix::factor;

namespace dp = ::datapod;

TEST_CASE("GaussianFactor - construction and basic access") {
    SUBCASE("Simple 1D factor") {
        dp::Vector<Key> keys = {1};
        GaussianFactor::Matrix J(1, 1);
        J(0, 0) = 2.0;
        dp::Vector<GaussianFactor::Matrix> jacobians = {J};
        GaussianFactor::Vector b(1);
        b[0] = 3.0;

        GaussianFactor gf(keys, jacobians, b);

        CHECK(gf.size() == 1);
        CHECK(gf.dim() == 1);
        CHECK(gf.b().size() == 1);
        CHECK(gf.b()[0] == 3.0);
        CHECK(gf.jacobians().size() == 1);
        CHECK(gf.jacobian(1)(0, 0) == 2.0);
    }

    SUBCASE("2D factor with 2 variables") {
        dp::Vector<Key> keys = {1, 2};

        GaussianFactor::Matrix J1(2, 3);
        J1(0, 0) = 1.0;
        J1(0, 1) = 2.0;
        J1(0, 2) = 3.0;
        J1(1, 0) = 4.0;
        J1(1, 1) = 5.0;
        J1(1, 2) = 6.0;

        GaussianFactor::Matrix J2(2, 3);
        J2(0, 0) = 7.0;
        J2(0, 1) = 8.0;
        J2(0, 2) = 9.0;
        J2(1, 0) = 10.0;
        J2(1, 1) = 11.0;
        J2(1, 2) = 12.0;

        dp::Vector<GaussianFactor::Matrix> jacobians = {J1, J2};
        GaussianFactor::Vector b(2);
        b[0] = 1.5;
        b[1] = 2.5;

        GaussianFactor gf(keys, jacobians, b);

        CHECK(gf.size() == 2);
        CHECK(gf.dim() == 2);
        CHECK(gf.jacobian(1)(0, 0) == 1.0);
        CHECK(gf.jacobian(2)(1, 2) == 12.0);
    }

    SUBCASE("Dimension mismatch throws") {
        dp::Vector<Key> keys = {1, 2};
        GaussianFactor::Matrix J1(2, 3);
        GaussianFactor::Matrix J2(3, 3); // Wrong dimension!
        dp::Vector<GaussianFactor::Matrix> jacobians = {J1, J2};
        GaussianFactor::Vector b(2);
        b[0] = 1.0;
        b[1] = 2.0;

        CHECK_THROWS_AS(GaussianFactor(keys, jacobians, b), std::invalid_argument);
    }

    SUBCASE("Key/Jacobian count mismatch throws") {
        dp::Vector<Key> keys = {1, 2, 3};
        GaussianFactor::Matrix J1(2, 3);
        GaussianFactor::Matrix J2(2, 3);
        dp::Vector<GaussianFactor::Matrix> jacobians = {J1, J2}; // Only 2 Jacobians for 3 keys!
        GaussianFactor::Vector b(2);
        b[0] = 1.0;
        b[1] = 2.0;

        CHECK_THROWS_AS(GaussianFactor(keys, jacobians, b), std::invalid_argument);
    }
}

TEST_CASE("GaussianFactor - error computation") {
    SUBCASE("Zero delta gives error from b") {
        dp::Vector<Key> keys = {1};
        GaussianFactor::Matrix J(1, 1);
        J(0, 0) = 2.0;
        dp::Vector<GaussianFactor::Matrix> jacobians = {J};
        GaussianFactor::Vector b(1);
        b[0] = 3.0;

        GaussianFactor gf(keys, jacobians, b);

        std::map<Key, GaussianFactor::Vector> deltas;
        GaussianFactor::Vector delta(1);
        delta[0] = 0.0;
        deltas[1] = delta;

        // error = 0.5 * ||b||^2 = 0.5 * 9 = 4.5
        double err = gf.error(deltas);
        CHECK(err == doctest::Approx(4.5));
    }

    SUBCASE("Non-zero delta") {
        dp::Vector<Key> keys = {1};
        GaussianFactor::Matrix J(1, 1);
        J(0, 0) = 2.0;
        dp::Vector<GaussianFactor::Matrix> jacobians = {J};
        GaussianFactor::Vector b(1);
        b[0] = 3.0;

        GaussianFactor gf(keys, jacobians, b);

        std::map<Key, GaussianFactor::Vector> deltas;
        GaussianFactor::Vector delta(1);
        delta[0] = 1.0;
        deltas[1] = delta;

        // error = 0.5 * ||J*dx + b||^2 = 0.5 * ||2*1 + 3||^2 = 0.5 * 25 = 12.5
        double err = gf.error(deltas);
        CHECK(err == doctest::Approx(12.5));
    }

    SUBCASE("Multi-dimensional error") {
        dp::Vector<Key> keys = {1};
        GaussianFactor::Matrix J(2, 2);
        J(0, 0) = 1.0;
        J(0, 1) = 0.0;
        J(1, 0) = 0.0;
        J(1, 1) = 1.0;
        dp::Vector<GaussianFactor::Matrix> jacobians = {J};
        GaussianFactor::Vector b(2);
        b[0] = 3.0;
        b[1] = 4.0;

        GaussianFactor gf(keys, jacobians, b);

        std::map<Key, GaussianFactor::Vector> deltas;
        GaussianFactor::Vector delta(2);
        delta[0] = 0.0;
        delta[1] = 0.0;
        deltas[1] = delta;

        // error = 0.5 * (3^2 + 4^2) = 0.5 * 25 = 12.5
        double err = gf.error(deltas);
        CHECK(err == doctest::Approx(12.5));
    }

    SUBCASE("Two variables") {
        dp::Vector<Key> keys = {1, 2};
        GaussianFactor::Matrix J1(1, 1);
        J1(0, 0) = 2.0;
        GaussianFactor::Matrix J2(1, 1);
        J2(0, 0) = 3.0;
        dp::Vector<GaussianFactor::Matrix> jacobians = {J1, J2};
        GaussianFactor::Vector b(1);
        b[0] = 1.0;

        GaussianFactor gf(keys, jacobians, b);

        std::map<Key, GaussianFactor::Vector> deltas;
        GaussianFactor::Vector delta1(1);
        delta1[0] = 1.0;
        deltas[1] = delta1;
        GaussianFactor::Vector delta2(1);
        delta2[0] = 1.0;
        deltas[2] = delta2;

        // error = 0.5 * ||2*1 + 3*1 + 1||^2 = 0.5 * 36 = 18
        double err = gf.error(deltas);
        CHECK(err == doctest::Approx(18.0));
    }
}

TEST_CASE("SE2PriorFactor - linearization") {
    SUBCASE("Linearize at prior") {
        SE2d prior(0, 0, 0); // SE2d(theta, x, y)
        Vec3d sigmas{1, 1, 1};
        SE2PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<SE2d>(1, SE2d(0, 0, 0)); // SE2d(theta, x, y)

        auto gf = factor.linearize(values);

        // At the prior, error should be zero
        CHECK(gf->dim() == 3);
        CHECK(gf->size() == 1);

        // Error vector should be zero
        const auto &b = gf->b();
        CHECK(b[0] == doctest::Approx(0.0).epsilon(0.01));
        CHECK(b[1] == doctest::Approx(0.0).epsilon(0.01));
        CHECK(b[2] == doctest::Approx(0.0).epsilon(0.01));

        // Jacobian should be approximately identity (for unit sigmas)
        const auto &J = gf->jacobian(1);
        CHECK(J.rows() == 3);
        CHECK(J.cols() == 3);

        // Diagonal should be approximately 1 (scaled by 1/sigma = 1)
        CHECK(J(0, 0) == doctest::Approx(1.0).epsilon(0.1));
        CHECK(J(1, 1) == doctest::Approx(1.0).epsilon(0.1));
        CHECK(J(2, 2) == doctest::Approx(1.0).epsilon(0.1));
    }

    SUBCASE("Linearize away from prior") {
        SE2d prior(0, 1, 2); // SE2d(theta, x, y) - theta=0, x=1, y=2
        Vec3d sigmas{0.1, 0.1, 0.1};
        SE2PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<SE2d>(1, SE2d(0.1, 1.5, 2.5)); // SE2d(theta, x, y) - theta=0.1, x=1.5, y=2.5

        auto gf = factor.linearize(values);

        // Error vector should be non-zero
        const auto &b = gf->b();
        CHECK(std::abs(b[0]) > 1.0); // Should have significant error
        CHECK(std::abs(b[1]) > 1.0);
        CHECK(std::abs(b[2]) > 0.5);

        // Jacobian should still be approximately diagonal
        const auto &J = gf->jacobian(1);
        CHECK(J(0, 0) == doctest::Approx(10.0).epsilon(1.0)); // 1/0.1 = 10
        CHECK(J(1, 1) == doctest::Approx(10.0).epsilon(1.0));
        CHECK(J(2, 2) == doctest::Approx(10.0).epsilon(1.0));
    }
}

TEST_CASE("SE2BetweenFactor - linearization") {
    SUBCASE("Linearize with identity measurement") {
        SE2d measured(0, 0, 0); // SE2d(theta, x, y) - No motion
        Vec3d sigmas{1, 1, 1};
        SE2BetweenFactor factor(1, 2, measured, sigmas);

        Values values;
        values.insert<SE2d>(1, SE2d(0, 0, 0)); // SE2d(theta, x, y)
        values.insert<SE2d>(2, SE2d(0, 0, 0)); // SE2d(theta, x, y)

        auto gf = factor.linearize(values);

        // At identity, error should be zero
        CHECK(gf->dim() == 3);
        CHECK(gf->size() == 2); // Two variables

        const auto &b = gf->b();
        CHECK(b[0] == doctest::Approx(0.0).epsilon(0.01));
        CHECK(b[1] == doctest::Approx(0.0).epsilon(0.01));
        CHECK(b[2] == doctest::Approx(0.0).epsilon(0.01));

        // Should have Jacobians for both keys
        CHECK(gf->jacobian(1).rows() == 3);
        CHECK(gf->jacobian(1).cols() == 3);
        CHECK(gf->jacobian(2).rows() == 3);
        CHECK(gf->jacobian(2).cols() == 3);
    }

    SUBCASE("Linearize with translation") {
        SE2d measured(0, 1, 0); // SE2d(theta, x, y) - Move forward 1 meter in x
        Vec3d sigmas{0.1, 0.1, 0.1};
        SE2BetweenFactor factor(1, 2, measured, sigmas);

        Values values;
        values.insert<SE2d>(1, SE2d(0, 0, 0)); // SE2d(theta, x, y)
        values.insert<SE2d>(2, SE2d(0, 1, 0)); // SE2d(theta, x, y) - Exactly at measurement

        auto gf = factor.linearize(values);

        const auto &b = gf->b();
        // Error should be approximately zero
        CHECK(std::abs(b[0]) < 0.1);
        CHECK(std::abs(b[1]) < 0.1);
        CHECK(std::abs(b[2]) < 0.1);
    }

    SUBCASE("Linearize with error") {
        SE2d measured(0, 1, 0); // SE2d(theta, x, y) - expected 1m forward in x
        Vec3d sigmas{0.1, 0.1, 0.1};
        SE2BetweenFactor factor(1, 2, measured, sigmas);

        Values values;
        values.insert<SE2d>(1, SE2d(0, 0, 0));       // SE2d(theta, x, y)
        values.insert<SE2d>(2, SE2d(0.1, 1.5, 0.2)); // SE2d(theta, x, y) - Deviated from measurement

        auto gf = factor.linearize(values);

        const auto &b = gf->b();
        // Error vector should be non-zero
        CHECK(std::abs(b[0]) > 1.0); // Should have significant x error
        CHECK(std::abs(b[1]) > 1.0); // Should have significant y error
    }
}

TEST_CASE("Linearization - numerical stability") {
    SUBCASE("Very small sigmas") {
        SE2d prior(0, 0, 0); // SE2d(theta, x, y)
        Vec3d sigmas{0.001, 0.001, 0.001};
        SE2PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<SE2d>(1, SE2d(0.0001, 0.0001, 0.0001)); // SE2d(theta, x, y)

        auto gf = factor.linearize(values);

        // Should still work without numerical issues
        CHECK(gf->dim() == 3);
        CHECK(std::isfinite(gf->b()[0]));
        CHECK(std::isfinite(gf->b()[1]));
        CHECK(std::isfinite(gf->b()[2]));
    }

    SUBCASE("Large values") {
        SE2d prior(3.14, 1000, 2000); // SE2d(theta, x, y) - theta=3.14, x=1000, y=2000
        Vec3d sigmas{10, 10, 0.1};
        SE2PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<SE2d>(1, SE2d(3.24, 1050, 2050)); // SE2d(theta, x, y) - theta=3.24, x=1050, y=2050

        auto gf = factor.linearize(values);

        // Should handle large values correctly
        CHECK(gf->dim() == 3);
        CHECK(std::isfinite(gf->b()[0]));
        CHECK(std::isfinite(gf->b()[1]));
        CHECK(std::isfinite(gf->b()[2]));
    }
}

TEST_CASE("Linearization - verify with analytical derivatives") {
    SUBCASE("SE2PriorFactor analytical check") {
        // For a prior factor with weighted residual r = log(prior^{-1} * pose) / sigma
        // The Jacobian is approximately dr/dx = 1/sigma * I (identity matrix) for small errors

        SE2d prior(0, 1, 2); // SE2d(theta, x, y) - theta=0, x=1, y=2
        Vec3d sigmas{0.5, 0.5, 0.5};
        SE2PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<SE2d>(1, SE2d(0.1, 1.1, 2.1)); // SE2d(theta, x, y) - theta=0.1, x=1.1, y=2.1

        auto gf = factor.linearize(values);
        const auto &J = gf->jacobian(1);

        // Analytical Jacobian should be approximately diag(1/sigma) = diag(2, 2, 2)
        CHECK(J(0, 0) == doctest::Approx(2.0).epsilon(0.2));
        CHECK(J(1, 1) == doctest::Approx(2.0).epsilon(0.2));
        CHECK(J(2, 2) == doctest::Approx(2.0).epsilon(0.2));
    }
}
