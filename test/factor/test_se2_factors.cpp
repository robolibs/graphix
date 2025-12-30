#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/se2_between_factor.hpp"
#include "graphix/factor/nonlinear/se2_prior_factor.hpp"
#include "graphix/factor/types.hpp"
#include "graphix/factor/values.hpp"
#include "graphix/kernel.hpp"
#include <doctest/doctest.h>

using namespace graphix;
using namespace graphix::factor;

// Note: SE2d constructor is SE2d(theta, tx, ty)
// SE2d::x() returns tx, SE2d::y() returns ty, SE2d::angle() returns theta

TEST_CASE("SE2PriorFactor - construction") {
    // SE2d(theta, tx, ty) - angle first, then translation
    SE2d prior(0.5, 1.0, 2.0); // theta=0.5, x=1.0, y=2.0
    Vec3d sigmas{0.1, 0.1, 0.1};

    SE2PriorFactor factor(X(0), prior, sigmas);

    CHECK(factor.keys().size() == 1);
    CHECK(factor.keys()[0] == X(0));
    CHECK(factor.prior().x() == doctest::Approx(1.0));
    CHECK(factor.prior().y() == doctest::Approx(2.0));
    CHECK(factor.prior().angle() == doctest::Approx(0.5));
    CHECK(factor.sigmas() == sigmas);
}

TEST_CASE("SE2PriorFactor - zero error at prior") {
    SE2d prior(M_PI / 4, 5.0, 10.0); // theta=pi/4, x=5, y=10
    Vec3d sigmas{1.0, 1.0, 1.0};

    SE2PriorFactor factor(X(0), prior, sigmas);

    Values values;
    values.insert(X(0), prior);

    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.0));
}

TEST_CASE("SE2PriorFactor - error with translation offset") {
    SE2d prior(0.0, 0.0, 0.0); // identity
    Vec3d sigmas{1.0, 1.0, 1.0};

    SE2PriorFactor factor(X(0), prior, sigmas);

    Values values;
    values.insert(X(0), SE2d(0.0, 1.0, 0.0)); // theta=0, x=1, y=0 (offset by 1 in x)

    // Error should be approximately 0.5 * 1^2 = 0.5
    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.5).epsilon(0.01));
}

TEST_CASE("SE2PriorFactor - error with sigma weighting") {
    SE2d prior(0.0, 0.0, 0.0);
    Vec3d sigmas{0.1, 0.1, 0.1}; // Small sigma = high confidence

    SE2PriorFactor factor(X(0), prior, sigmas);

    Values values;
    values.insert(X(0), SE2d(0.0, 1.0, 0.0)); // theta=0, x=1, y=0

    // Error = 0.5 * ((1-0)/0.1)^2 = 0.5 * 100 = 50
    double error = factor.error(values);
    CHECK(error == doctest::Approx(50.0).epsilon(1.0));
}

TEST_CASE("SE2PriorFactor - multi-dimensional error") {
    SE2d prior(0.0, 1.0, 2.0); // theta=0, x=1, y=2
    Vec3d sigmas{1.0, 1.0, 1.0};

    SE2PriorFactor factor(X(0), prior, sigmas);

    Values values;
    values.insert(X(0), SE2d(0.0, 2.0, 3.0)); // theta=0, x=2, y=3 (each dimension off by 1)

    // Error should be approximately 0.5 * (1^2 + 1^2) = 1.0
    double error = factor.error(values);
    CHECK(error == doctest::Approx(1.0).epsilon(0.1));
}

TEST_CASE("SE2PriorFactor - invalid sigma throws") {
    SE2d prior(0.0, 0.0, 0.0);
    Vec3d bad_sigmas{0.0, 1.0, 1.0}; // Zero sigma

    CHECK_THROWS_AS(SE2PriorFactor(X(0), prior, bad_sigmas), std::invalid_argument);
}

TEST_CASE("SE2PriorFactor - negative sigma throws") {
    SE2d prior(0.0, 0.0, 0.0);
    Vec3d bad_sigmas{1.0, -1.0, 1.0}; // Negative sigma

    CHECK_THROWS_AS(SE2PriorFactor(X(0), prior, bad_sigmas), std::invalid_argument);
}

TEST_CASE("SE2BetweenFactor - construction") {
    SE2d measured(0.0, 1.0, 0.0); // theta=0, x=1, y=0 (move 1m forward)
    Vec3d sigmas{0.1, 0.1, 0.1};

    SE2BetweenFactor factor(X(0), X(1), measured, sigmas);

    CHECK(factor.keys().size() == 2);
    CHECK(factor.keys()[0] == X(0));
    CHECK(factor.keys()[1] == X(1));
    CHECK(factor.measured().x() == doctest::Approx(1.0));
    CHECK(factor.sigmas() == sigmas);
}

TEST_CASE("SE2BetweenFactor - zero error at measurement") {
    SE2d measured(0.0, 1.0, 0.0); // Move 1m forward
    Vec3d sigmas{1.0, 1.0, 1.0};

    SE2BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), SE2d(0.0, 0.0, 0.0)); // identity
    values.insert(X(1), SE2d(0.0, 1.0, 0.0)); // T_i^{-1} * T_j = measured

    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.0).epsilon(0.001));
}

TEST_CASE("SE2BetweenFactor - error with offset") {
    SE2d measured(0.0, 1.0, 0.0); // Move 1m forward
    Vec3d sigmas{1.0, 1.0, 1.0};

    SE2BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), SE2d(0.0, 0.0, 0.0));
    values.insert(X(1), SE2d(0.0, 2.0, 0.0)); // Actual diff is 2, measured is 1

    // Error should be approximately 0.5 * 1^2 = 0.5
    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.5).epsilon(0.1));
}

TEST_CASE("SE2BetweenFactor - odometry example") {
    // Robot moves from (0,0,0) to (1,0,0) then turns 90 degrees
    SE2d measured(M_PI / 2, 1.0, 0.0); // theta=pi/2, x=1, y=0
    Vec3d sigmas{0.1, 0.1, 0.1};

    SE2BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), SE2d(0.0, 0.0, 0.0));
    values.insert(X(1), SE2d(M_PI / 2, 1.0, 0.0));

    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("SE2BetweenFactor - error with sigma weighting") {
    SE2d measured(0.0, 1.0, 0.0);
    Vec3d sigmas{0.1, 0.1, 0.1};

    SE2BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), SE2d(0.0, 0.0, 0.0));
    values.insert(X(1), SE2d(0.0, 2.0, 0.0)); // Off by 1

    // Error = 0.5 * ((1)/0.1)^2 = 50
    double error = factor.error(values);
    CHECK(error == doctest::Approx(50.0).epsilon(5.0));
}

TEST_CASE("SE2BetweenFactor - invalid sigma throws") {
    SE2d measured(0.0, 1.0, 0.0);
    Vec3d bad_sigmas{0.0, 1.0, 1.0};

    CHECK_THROWS_AS(SE2BetweenFactor(X(0), X(1), measured, bad_sigmas), std::invalid_argument);
}

TEST_CASE("SE2 factors in graph") {
    Graph<NonlinearFactor> graph;

    graph.add(std::make_shared<SE2PriorFactor>(X(0), SE2d(0.0, 0.0, 0.0), Vec3d{0.1, 0.1, 0.1}));
    graph.add(std::make_shared<SE2BetweenFactor>(X(0), X(1), SE2d(0.0, 1.0, 0.0), Vec3d{0.1, 0.1, 0.1}));

    CHECK(graph.size() == 2);

    Values values;
    values.insert(X(0), SE2d(0.0, 0.0, 0.0));
    values.insert(X(1), SE2d(0.0, 1.0, 0.0));

    // Compute total error
    double total_error = 0.0;
    for (const auto &factor : graph) {
        total_error += factor->error(values);
    }

    CHECK(total_error == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("SE2 factors - simple chain") {
    // Chain: x0=origin -> x1=(1,0) -> x2=(2,0)
    Graph<NonlinearFactor> graph;

    graph.add(std::make_shared<SE2PriorFactor>(X(0), SE2d(0.0, 0.0, 0.0), Vec3d{0.1, 0.1, 0.1}));
    graph.add(std::make_shared<SE2BetweenFactor>(X(0), X(1), SE2d(0.0, 1.0, 0.0), Vec3d{0.1, 0.1, 0.1}));
    graph.add(std::make_shared<SE2BetweenFactor>(X(1), X(2), SE2d(0.0, 1.0, 0.0), Vec3d{0.1, 0.1, 0.1}));

    Values values;
    values.insert(X(0), SE2d(0.0, 0.0, 0.0));
    values.insert(X(1), SE2d(0.0, 1.0, 0.0));
    values.insert(X(2), SE2d(0.0, 2.0, 0.0));

    double total_error = 0.0;
    for (const auto &factor : graph) {
        total_error += factor->error(values);
    }

    CHECK(total_error == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("SE2 factors - 2D pose with rotation") {
    // Robot moves forward then turns
    Graph<NonlinearFactor> graph;

    // Start at origin
    graph.add(std::make_shared<SE2PriorFactor>(X(0), SE2d(0.0, 0.0, 0.0), Vec3d{0.1, 0.1, 0.05}));

    // Move forward 1m (in local frame)
    graph.add(std::make_shared<SE2BetweenFactor>(X(0), X(1), SE2d(0.0, 1.0, 0.0), Vec3d{0.2, 0.2, 0.1}));

    // Turn 90 degrees (pure rotation)
    graph.add(std::make_shared<SE2BetweenFactor>(X(1), X(2), SE2d(M_PI / 2, 0.0, 0.0), Vec3d{0.2, 0.2, 0.1}));

    Values values;
    values.insert(X(0), SE2d(0.0, 0.0, 0.0));
    values.insert(X(1), SE2d(0.0, 1.0, 0.0));
    values.insert(X(2), SE2d(M_PI / 2, 1.0, 0.0));

    double total_error = 0.0;
    for (const auto &factor : graph) {
        total_error += factor->error(values);
    }

    CHECK(total_error == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("SE2BetweenFactor - translation is in local frame") {
    // Pose i faces +Y (90 degrees). A 1m forward motion in local frame results in +Y in world.
    SE2d measured(0.0, 1.0, 0.0); // 1m forward in local frame
    Vec3d sigmas{1.0, 1.0, 1.0};

    SE2BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), SE2d(M_PI / 2, 0.0, 0.0)); // At origin, facing +Y (theta=pi/2)
    values.insert(X(1), SE2d(M_PI / 2, 0.0, 1.0)); // Moved 1m in +Y direction

    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("SE2 factors - get keys from graph") {
    Graph<NonlinearFactor> graph;

    graph.add(std::make_shared<SE2PriorFactor>(X(0), SE2d(0, 0, 0), Vec3d{1, 1, 1}));
    graph.add(std::make_shared<SE2BetweenFactor>(X(0), X(1), SE2d(0, 1, 0), Vec3d{1, 1, 1}));
    graph.add(std::make_shared<SE2BetweenFactor>(X(1), X(2), SE2d(0, 1, 0), Vec3d{1, 1, 1}));

    auto keys = graph.keys();

    CHECK(keys.size() == 3);
    CHECK(std::find(keys.begin(), keys.end(), X(0)) != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), X(1)) != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), X(2)) != keys.end());
}

TEST_CASE("SE2 factors - loop closure") {
    // Square path with loop closure
    Graph<NonlinearFactor> graph;

    Vec3d odom_sigma{0.1, 0.1, 0.05};
    Vec3d prior_sigma{0.01, 0.01, 0.01};

    // Prior at origin
    graph.add(std::make_shared<SE2PriorFactor>(X(0), SE2d(0.0, 0.0, 0.0), prior_sigma));

    // Square path: right, up, left, down (all in local frame)
    graph.add(std::make_shared<SE2BetweenFactor>(X(0), X(1), SE2d(0.0, 2.0, 0.0), odom_sigma));      // move 2m forward
    graph.add(std::make_shared<SE2BetweenFactor>(X(1), X(2), SE2d(M_PI / 2, 2.0, 0.0), odom_sigma)); // turn + move
    graph.add(std::make_shared<SE2BetweenFactor>(X(2), X(3), SE2d(M_PI / 2, 2.0, 0.0), odom_sigma)); // turn + move
    graph.add(std::make_shared<SE2BetweenFactor>(X(3), X(4), SE2d(M_PI / 2, 2.0, 0.0), odom_sigma)); // turn + move

    // Loop closure
    graph.add(std::make_shared<SE2BetweenFactor>(X(4), X(0), SE2d(M_PI / 2, 0.0, 0.0), odom_sigma));

    CHECK(graph.size() == 6);
}
