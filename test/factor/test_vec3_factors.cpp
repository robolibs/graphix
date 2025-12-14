#include "doctest.h"
#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/vec3_between_factor.hpp"
#include "graphix/factor/nonlinear/vec3_prior_factor.hpp"
#include "graphix/factor/types/vec3d.hpp"
#include "graphix/factor/values.hpp"
#include "graphix/kernel.hpp"

using namespace graphix;
using namespace graphix::factor;

TEST_CASE("Vec3PriorFactor - construction") {
    Vec3d prior(1.0, 2.0, 3.0);
    Vec3d sigmas(0.1, 0.1, 0.1);

    Vec3PriorFactor factor(X(0), prior, sigmas);

    CHECK(factor.keys().size() == 1);
    CHECK(factor.keys()[0] == X(0));
    CHECK(factor.prior() == prior);
    CHECK(factor.sigmas() == sigmas);
}

TEST_CASE("Vec3PriorFactor - zero error at prior") {
    Vec3d prior(5.0, 10.0, 15.0);
    Vec3d sigmas(1.0, 1.0, 1.0);

    Vec3PriorFactor factor(X(0), prior, sigmas);

    Values values;
    values.insert(X(0), prior);

    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.0));
}

TEST_CASE("Vec3PriorFactor - error with offset") {
    Vec3d prior(0.0, 0.0, 0.0);
    Vec3d sigmas(1.0, 1.0, 1.0);

    Vec3PriorFactor factor(X(0), prior, sigmas);

    Values values;
    values.insert(X(0), Vec3d(1.0, 0.0, 0.0)); // Offset by 1 in x

    // Error = 0.5 * ((1-0)/1)^2 = 0.5
    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.5));
}

TEST_CASE("Vec3PriorFactor - error with sigma weighting") {
    Vec3d prior(0.0, 0.0, 0.0);
    Vec3d sigmas(0.1, 0.1, 0.1); // Small sigma = high confidence

    Vec3PriorFactor factor(X(0), prior, sigmas);

    Values values;
    values.insert(X(0), Vec3d(1.0, 0.0, 0.0));

    // Error = 0.5 * ((1-0)/0.1)^2 = 0.5 * 100 = 50
    double error = factor.error(values);
    CHECK(error == doctest::Approx(50.0));
}

TEST_CASE("Vec3PriorFactor - multi-dimensional error") {
    Vec3d prior(1.0, 2.0, 3.0);
    Vec3d sigmas(1.0, 1.0, 1.0);

    Vec3PriorFactor factor(X(0), prior, sigmas);

    Values values;
    values.insert(X(0), Vec3d(2.0, 3.0, 4.0)); // Each dimension off by 1

    // Error = 0.5 * (1^2 + 1^2 + 1^2) = 1.5
    double error = factor.error(values);
    CHECK(error == doctest::Approx(1.5));
}

TEST_CASE("Vec3PriorFactor - invalid sigma throws") {
    Vec3d prior(0.0, 0.0, 0.0);
    Vec3d bad_sigmas(0.0, 1.0, 1.0); // Zero sigma

    CHECK_THROWS_AS(Vec3PriorFactor(X(0), prior, bad_sigmas), std::invalid_argument);
}

TEST_CASE("Vec3PriorFactor - negative sigma throws") {
    Vec3d prior(0.0, 0.0, 0.0);
    Vec3d bad_sigmas(1.0, -1.0, 1.0); // Negative sigma

    CHECK_THROWS_AS(Vec3PriorFactor(X(0), prior, bad_sigmas), std::invalid_argument);
}

TEST_CASE("Vec3BetweenFactor - construction") {
    Vec3d measured(1.0, 0.0, 0.0);
    Vec3d sigmas(0.1, 0.1, 0.1);

    Vec3BetweenFactor factor(X(0), X(1), measured, sigmas);

    CHECK(factor.keys().size() == 2);
    CHECK(factor.keys()[0] == X(0));
    CHECK(factor.keys()[1] == X(1));
    CHECK(factor.measured() == measured);
    CHECK(factor.sigmas() == sigmas);
}

TEST_CASE("Vec3BetweenFactor - zero error at measurement") {
    Vec3d measured(1.0, 2.0, 3.0);
    Vec3d sigmas(1.0, 1.0, 1.0);

    Vec3BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), Vec3d(0.0, 0.0, 0.0));
    values.insert(X(1), Vec3d(1.0, 2.0, 3.0)); // vj - vi = measured

    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.0));
}

TEST_CASE("Vec3BetweenFactor - error with offset") {
    Vec3d measured(1.0, 0.0, 0.0);
    Vec3d sigmas(1.0, 1.0, 1.0);

    Vec3BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), Vec3d(0.0, 0.0, 0.0));
    values.insert(X(1), Vec3d(2.0, 0.0, 0.0)); // Actual diff is 2, measured is 1

    // Error = 0.5 * ((2-1)/1)^2 = 0.5
    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.5));
}

TEST_CASE("Vec3BetweenFactor - odometry example") {
    // Robot moves from (0,0,0°) to (1,0,90°)
    Vec3d measured(1.0, 0.0, M_PI / 2);
    Vec3d sigmas(0.1, 0.1, 0.1);

    Vec3BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), Vec3d(0.0, 0.0, 0.0));
    values.insert(X(1), Vec3d(1.0, 0.0, M_PI / 2));

    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.0).epsilon(0.001));
}

TEST_CASE("Vec3BetweenFactor - error with sigma weighting") {
    Vec3d measured(1.0, 0.0, 0.0);
    Vec3d sigmas(0.1, 0.1, 0.1);

    Vec3BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), Vec3d(0.0, 0.0, 0.0));
    values.insert(X(1), Vec3d(2.0, 0.0, 0.0)); // Off by 1

    // Error = 0.5 * ((1)/0.1)^2 = 50
    double error = factor.error(values);
    CHECK(error == doctest::Approx(50.0));
}

TEST_CASE("Vec3BetweenFactor - different sigma per dimension") {
    Vec3d measured(1.0, 0.0, 0.0);
    Vec3d sigmas(0.1, 1.0, 1.0); // x is more certain

    Vec3BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), Vec3d(0.0, 0.0, 0.0));
    values.insert(X(1), Vec3d(2.0, 1.0, 0.0)); // x off by 1, y off by 1

    // Error = 0.5 * ((1/0.1)^2 + (1/1.0)^2 + 0) = 0.5 * (100 + 1) = 50.5
    double error = factor.error(values);
    CHECK(error == doctest::Approx(50.5));
}

TEST_CASE("Vec3BetweenFactor - invalid sigma throws") {
    Vec3d measured(1.0, 0.0, 0.0);
    Vec3d bad_sigmas(0.0, 1.0, 1.0);

    CHECK_THROWS_AS(Vec3BetweenFactor(X(0), X(1), measured, bad_sigmas), std::invalid_argument);
}

TEST_CASE("Vec3 factors in graph") {
    Graph<NonlinearFactor> graph;

    graph.add(std::make_shared<Vec3PriorFactor>(X(0), Vec3d(0.0, 0.0, 0.0), Vec3d(0.1, 0.1, 0.1)));
    graph.add(std::make_shared<Vec3BetweenFactor>(X(0), X(1), Vec3d(1.0, 0.0, 0.0), Vec3d(0.1, 0.1, 0.1)));

    CHECK(graph.size() == 2);

    Values values;
    values.insert(X(0), Vec3d(0.0, 0.0, 0.0));
    values.insert(X(1), Vec3d(1.0, 0.0, 0.0));

    // Compute total error
    double total_error = 0.0;
    for (const auto &factor : graph) {
        total_error += factor->error(values);
    }

    CHECK(total_error == doctest::Approx(0.0));
}

TEST_CASE("Vec3 factors - simple chain") {
    // Chain: x0=0 -> x1=1 -> x2=2
    Graph<NonlinearFactor> graph;

    graph.add(std::make_shared<Vec3PriorFactor>(X(0), Vec3d(0.0, 0.0, 0.0), Vec3d(0.1, 0.1, 0.1)));
    graph.add(std::make_shared<Vec3BetweenFactor>(X(0), X(1), Vec3d(1.0, 0.0, 0.0), Vec3d(0.1, 0.1, 0.1)));
    graph.add(std::make_shared<Vec3BetweenFactor>(X(1), X(2), Vec3d(1.0, 0.0, 0.0), Vec3d(0.1, 0.1, 0.1)));

    Values values;
    values.insert(X(0), Vec3d(0.0, 0.0, 0.0));
    values.insert(X(1), Vec3d(1.0, 0.0, 0.0));
    values.insert(X(2), Vec3d(2.0, 0.0, 0.0));

    double total_error = 0.0;
    for (const auto &factor : graph) {
        total_error += factor->error(values);
    }

    CHECK(total_error == doctest::Approx(0.0));
}

TEST_CASE("Vec3 factors - 2D pose interpretation") {
    // Interpret Vec3d as (x, y, theta) pose
    Graph<NonlinearFactor> graph;

    // Start at origin
    Vec3d origin(0.0, 0.0, 0.0);
    graph.add(std::make_shared<Vec3PriorFactor>(X(0), origin, Vec3d(0.1, 0.1, 0.05)));

    // Move forward 1m
    Vec3d odom1(1.0, 0.0, 0.0);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(0), X(1), odom1, Vec3d(0.2, 0.2, 0.1)));

    // Turn 90° right
    Vec3d odom2(0.0, 0.0, M_PI / 2);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(1), X(2), odom2, Vec3d(0.2, 0.2, 0.1)));

    Values values;
    values.insert(X(0), Vec3d(0.0, 0.0, 0.0));
    values.insert(X(1), Vec3d(1.0, 0.0, 0.0));
    values.insert(X(2), Vec3d(1.0, 0.0, M_PI / 2));

    double total_error = 0.0;
    for (const auto &factor : graph) {
        total_error += factor->error(values);
    }

    CHECK(total_error == doctest::Approx(0.0).epsilon(0.001));
}

TEST_CASE("Vec3BetweenFactor - negative relative measurement") {
    Vec3d measured(-1.0, -2.0, -3.0);
    Vec3d sigmas(1.0, 1.0, 1.0);

    Vec3BetweenFactor factor(X(0), X(1), measured, sigmas);

    Values values;
    values.insert(X(0), Vec3d(5.0, 5.0, 5.0));
    values.insert(X(1), Vec3d(4.0, 3.0, 2.0)); // vj - vi = (-1, -2, -3)

    double error = factor.error(values);
    CHECK(error == doctest::Approx(0.0));
}

TEST_CASE("Vec3 factors - get keys from graph") {
    Graph<NonlinearFactor> graph;

    graph.add(std::make_shared<Vec3PriorFactor>(X(0), Vec3d(0, 0, 0), Vec3d(1, 1, 1)));
    graph.add(std::make_shared<Vec3BetweenFactor>(X(0), X(1), Vec3d(1, 0, 0), Vec3d(1, 1, 1)));
    graph.add(std::make_shared<Vec3BetweenFactor>(X(1), X(2), Vec3d(1, 0, 0), Vec3d(1, 1, 1)));

    auto keys = graph.keys();

    CHECK(keys.size() == 3);
    CHECK(std::find(keys.begin(), keys.end(), X(0)) != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), X(1)) != keys.end());
    CHECK(std::find(keys.begin(), keys.end(), X(2)) != keys.end());
}
