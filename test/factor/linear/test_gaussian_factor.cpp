#include "graphix/factor/linear/gaussian_factor.hpp"
#include "graphix/factor/nonlinear/vec3_between_factor.hpp"
#include "graphix/factor/nonlinear/vec3_prior_factor.hpp"
#include "graphix/factor/types/vec3d.hpp"
#include <doctest/doctest.h>

using namespace graphix;
using namespace graphix::factor;

TEST_CASE("GaussianFactor - construction and basic access") {
    SUBCASE("Simple 1D factor") {
        std::vector<Key> keys = {1};
        Matrix J(1, 1);
        J(0, 0) = 2.0;
        std::vector<Matrix> jacobians = {J};
        std::vector<double> b = {3.0};

        GaussianFactor gf(keys, jacobians, b);

        CHECK(gf.size() == 1);
        CHECK(gf.dim() == 1);
        CHECK(gf.b().size() == 1);
        CHECK(gf.b()[0] == 3.0);
        CHECK(gf.jacobians().size() == 1);
        CHECK(gf.jacobian(1)(0, 0) == 2.0);
    }

    SUBCASE("2D factor with 2 variables") {
        std::vector<Key> keys = {1, 2};

        Matrix J1(2, 3);
        J1(0, 0) = 1.0;
        J1(0, 1) = 2.0;
        J1(0, 2) = 3.0;
        J1(1, 0) = 4.0;
        J1(1, 1) = 5.0;
        J1(1, 2) = 6.0;

        Matrix J2(2, 3);
        J2(0, 0) = 7.0;
        J2(0, 1) = 8.0;
        J2(0, 2) = 9.0;
        J2(1, 0) = 10.0;
        J2(1, 1) = 11.0;
        J2(1, 2) = 12.0;

        std::vector<Matrix> jacobians = {J1, J2};
        std::vector<double> b = {1.5, 2.5};

        GaussianFactor gf(keys, jacobians, b);

        CHECK(gf.size() == 2);
        CHECK(gf.dim() == 2);
        CHECK(gf.jacobian(1)(0, 0) == 1.0);
        CHECK(gf.jacobian(2)(1, 2) == 12.0);
    }

    SUBCASE("Dimension mismatch throws") {
        std::vector<Key> keys = {1, 2};
        Matrix J1(2, 3);
        Matrix J2(3, 3); // Wrong dimension!
        std::vector<Matrix> jacobians = {J1, J2};
        std::vector<double> b = {1.0, 2.0};

        CHECK_THROWS_AS(GaussianFactor(keys, jacobians, b), std::invalid_argument);
    }

    SUBCASE("Key/Jacobian count mismatch throws") {
        std::vector<Key> keys = {1, 2, 3};
        Matrix J1(2, 3);
        Matrix J2(2, 3);
        std::vector<Matrix> jacobians = {J1, J2}; // Only 2 Jacobians for 3 keys!
        std::vector<double> b = {1.0, 2.0};

        CHECK_THROWS_AS(GaussianFactor(keys, jacobians, b), std::invalid_argument);
    }
}

TEST_CASE("GaussianFactor - error computation") {
    SUBCASE("Zero delta gives error from b") {
        std::vector<Key> keys = {1};
        Matrix J(1, 1);
        J(0, 0) = 2.0;
        std::vector<Matrix> jacobians = {J};
        std::vector<double> b = {3.0};

        GaussianFactor gf(keys, jacobians, b);

        std::map<Key, std::vector<double>> deltas;
        deltas[1] = {0.0};

        // error = 0.5 * ||b||^2 = 0.5 * 9 = 4.5
        double err = gf.error(deltas);
        CHECK(err == doctest::Approx(4.5));
    }

    SUBCASE("Non-zero delta") {
        std::vector<Key> keys = {1};
        Matrix J(1, 1);
        J(0, 0) = 2.0;
        std::vector<Matrix> jacobians = {J};
        std::vector<double> b = {3.0};

        GaussianFactor gf(keys, jacobians, b);

        std::map<Key, std::vector<double>> deltas;
        deltas[1] = {1.0};

        // error = 0.5 * ||J*dx + b||^2 = 0.5 * ||2*1 + 3||^2 = 0.5 * 25 = 12.5
        double err = gf.error(deltas);
        CHECK(err == doctest::Approx(12.5));
    }

    SUBCASE("Multi-dimensional error") {
        std::vector<Key> keys = {1};
        Matrix J(2, 2);
        J(0, 0) = 1.0;
        J(0, 1) = 0.0;
        J(1, 0) = 0.0;
        J(1, 1) = 1.0;
        std::vector<Matrix> jacobians = {J};
        std::vector<double> b = {3.0, 4.0};

        GaussianFactor gf(keys, jacobians, b);

        std::map<Key, std::vector<double>> deltas;
        deltas[1] = {0.0, 0.0};

        // error = 0.5 * (3^2 + 4^2) = 0.5 * 25 = 12.5
        double err = gf.error(deltas);
        CHECK(err == doctest::Approx(12.5));
    }

    SUBCASE("Two variables") {
        std::vector<Key> keys = {1, 2};
        Matrix J1(1, 1);
        J1(0, 0) = 2.0;
        Matrix J2(1, 1);
        J2(0, 0) = 3.0;
        std::vector<Matrix> jacobians = {J1, J2};
        std::vector<double> b = {1.0};

        GaussianFactor gf(keys, jacobians, b);

        std::map<Key, std::vector<double>> deltas;
        deltas[1] = {1.0};
        deltas[2] = {1.0};

        // error = 0.5 * ||2*1 + 3*1 + 1||^2 = 0.5 * 36 = 18
        double err = gf.error(deltas);
        CHECK(err == doctest::Approx(18.0));
    }
}

TEST_CASE("Vec3PriorFactor - linearization") {
    SUBCASE("Linearize at zero") {
        Vec3d prior(0, 0, 0);
        Vec3d sigmas(1, 1, 1);
        Vec3PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<Vec3d>(1, Vec3d(0, 0, 0));

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
        CHECK(J(0, 0) == doctest::Approx(1.0).epsilon(0.01));
        CHECK(J(1, 1) == doctest::Approx(1.0).epsilon(0.01));
        CHECK(J(2, 2) == doctest::Approx(1.0).epsilon(0.01));

        // Off-diagonals should be approximately 0
        CHECK(std::abs(J(0, 1)) < 0.01);
        CHECK(std::abs(J(0, 2)) < 0.01);
        CHECK(std::abs(J(1, 0)) < 0.01);
        CHECK(std::abs(J(1, 2)) < 0.01);
        CHECK(std::abs(J(2, 0)) < 0.01);
        CHECK(std::abs(J(2, 1)) < 0.01);
    }

    SUBCASE("Linearize away from prior") {
        Vec3d prior(1, 2, 0);
        Vec3d sigmas(0.1, 0.1, 0.1);
        Vec3PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<Vec3d>(1, Vec3d(1.5, 2.5, 0.1));

        auto gf = factor.linearize(values);

        // Error vector should be non-zero: (current - prior) / sigma
        const auto &b = gf->b();
        CHECK(b[0] == doctest::Approx(5.0).epsilon(0.1)); // (1.5 - 1.0) / 0.1 = 5
        CHECK(b[1] == doctest::Approx(5.0).epsilon(0.1)); // (2.5 - 2.0) / 0.1 = 5
        CHECK(b[2] == doctest::Approx(1.0).epsilon(0.1)); // (0.1 - 0.0) / 0.1 = 1

        // Jacobian should still be approximately diagonal
        const auto &J = gf->jacobian(1);
        CHECK(J(0, 0) == doctest::Approx(10.0).epsilon(0.1)); // 1/0.1 = 10
        CHECK(J(1, 1) == doctest::Approx(10.0).epsilon(0.1));
        CHECK(J(2, 2) == doctest::Approx(10.0).epsilon(0.1));
    }
}

TEST_CASE("Vec3BetweenFactor - linearization") {
    SUBCASE("Linearize with identity measurement") {
        Vec3d measured(0, 0, 0); // No motion
        Vec3d sigmas(1, 1, 1);
        Vec3BetweenFactor factor(1, 2, measured, sigmas);

        Values values;
        values.insert<Vec3d>(1, Vec3d(0, 0, 0));
        values.insert<Vec3d>(2, Vec3d(0, 0, 0));

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
        Vec3d measured(1, 0, 0); // Move forward 1 meter
        Vec3d sigmas(0.1, 0.1, 0.1);
        Vec3BetweenFactor factor(1, 2, measured, sigmas);

        Values values;
        values.insert<Vec3d>(1, Vec3d(0, 0, 0));
        values.insert<Vec3d>(2, Vec3d(1, 0, 0)); // Exactly at measurement

        auto gf = factor.linearize(values);

        const auto &b = gf->b();
        // Error should be approximately zero
        CHECK(std::abs(b[0]) < 0.1);
        CHECK(std::abs(b[1]) < 0.1);
        CHECK(std::abs(b[2]) < 0.1);
    }

    SUBCASE("Linearize with error") {
        Vec3d measured(1, 0, 0);
        Vec3d sigmas(0.1, 0.1, 0.1);
        Vec3BetweenFactor factor(1, 2, measured, sigmas);

        Values values;
        values.insert<Vec3d>(1, Vec3d(0, 0, 0));
        values.insert<Vec3d>(2, Vec3d(1.5, 0.2, 0.1)); // Deviated from measurement

        auto gf = factor.linearize(values);

        const auto &b = gf->b();
        // Error vector should be non-zero
        // Predicted is (1.5, 0.2, 0.1) - (0, 0, 0) = (1.5, 0.2, 0.1) in world frame
        // Measured is (1, 0, 0) in local frame
        // Residual = (predicted - measured) / sigma
        CHECK(std::abs(b[0]) > 1.0); // Should have significant x error
        CHECK(std::abs(b[1]) > 1.0); // Should have significant y error
    }
}

TEST_CASE("Linearization - numerical stability") {
    SUBCASE("Very small sigmas") {
        Vec3d prior(0, 0, 0);
        Vec3d sigmas(0.001, 0.001, 0.001);
        Vec3PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<Vec3d>(1, Vec3d(0.0001, 0.0001, 0.0001));

        auto gf = factor.linearize(values);

        // Should still work without numerical issues
        CHECK(gf->dim() == 3);
        CHECK(std::isfinite(gf->b()[0]));
        CHECK(std::isfinite(gf->b()[1]));
        CHECK(std::isfinite(gf->b()[2]));
    }

    SUBCASE("Large values") {
        Vec3d prior(1000, 2000, 3.14);
        Vec3d sigmas(10, 10, 0.1);
        Vec3PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<Vec3d>(1, Vec3d(1050, 2050, 3.24));

        auto gf = factor.linearize(values);

        // Should handle large values correctly
        CHECK(gf->dim() == 3);
        CHECK(std::isfinite(gf->b()[0]));
        CHECK(std::isfinite(gf->b()[1]));
        CHECK(std::isfinite(gf->b()[2]));
    }
}

TEST_CASE("Linearization - verify with analytical derivatives") {
    SUBCASE("Vec3PriorFactor analytical check") {
        // For a prior factor with weighted residual r = (x - prior) / sigma
        // The Jacobian is dr/dx = 1/sigma * I (identity matrix)

        Vec3d prior(1, 2, 3);
        Vec3d sigmas(0.5, 0.5, 0.5);
        Vec3PriorFactor factor(1, prior, sigmas);

        Values values;
        values.insert<Vec3d>(1, Vec3d(1.1, 2.1, 3.1));

        auto gf = factor.linearize(values);
        const auto &J = gf->jacobian(1);

        // Analytical Jacobian should be diag(1/sigma) = diag(2, 2, 2)
        CHECK(J(0, 0) == doctest::Approx(2.0).epsilon(0.01));
        CHECK(J(1, 1) == doctest::Approx(2.0).epsilon(0.01));
        CHECK(J(2, 2) == doctest::Approx(2.0).epsilon(0.01));

        // Off-diagonals should be zero
        CHECK(std::abs(J(0, 1)) < 0.01);
        CHECK(std::abs(J(0, 2)) < 0.01);
        CHECK(std::abs(J(1, 0)) < 0.01);
        CHECK(std::abs(J(1, 2)) < 0.01);
        CHECK(std::abs(J(2, 0)) < 0.01);
        CHECK(std::abs(J(2, 1)) < 0.01);
    }
}
