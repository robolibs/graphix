#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/between_factor.hpp"
#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/nonlinear/prior_factor.hpp"
#include "graphix/kernel.hpp"
#include <cmath>
#include <doctest/doctest.h>

using namespace graphix;
using namespace graphix::factor;

TEST_CASE("PriorFactor - construction") {
    PriorFactor prior(X(0), 10.0, 1.0);

    CHECK(prior.size() == 1);
    CHECK(prior.keys()[0] == X(0));
    CHECK(prior.prior() == 10.0);
    CHECK(prior.sigma() == 1.0);
}

TEST_CASE("PriorFactor - zero error at prior") {
    PriorFactor prior(X(0), 5.0, 1.0);

    Values values;
    values.insert(X(0), 5.0); // Exact prior

    double err = prior.error(values);
    CHECK(err == doctest::Approx(0.0));
}

TEST_CASE("PriorFactor - error with offset") {
    PriorFactor prior(X(0), 10.0, 1.0);

    Values values;
    values.insert(X(0), 11.0); // Off by 1 sigma

    // Error = 0.5 * ((11 - 10) / 1)^2 = 0.5 * 1 = 0.5
    double err = prior.error(values);
    CHECK(err == doctest::Approx(0.5));
}

TEST_CASE("PriorFactor - error scales with sigma") {
    PriorFactor prior1(X(0), 10.0, 1.0);
    PriorFactor prior2(X(0), 10.0, 2.0); // Larger sigma = less certain

    Values values;
    values.insert(X(0), 12.0); // Off by 2 units

    double err1 = prior1.error(values);
    double err2 = prior2.error(values);

    // Error1 = 0.5 * (2/1)^2 = 2.0
    // Error2 = 0.5 * (2/2)^2 = 0.5
    CHECK(err1 == doctest::Approx(2.0));
    CHECK(err2 == doctest::Approx(0.5));
}

TEST_CASE("PriorFactor - negative sigma throws") {
    CHECK_THROWS_AS(PriorFactor(X(0), 0.0, -1.0), std::invalid_argument);
}

TEST_CASE("PriorFactor - zero sigma throws") { CHECK_THROWS_AS(PriorFactor(X(0), 0.0, 0.0), std::invalid_argument); }

TEST_CASE("BetweenFactor - construction") {
    BetweenFactor between(X(0), X(1), 5.0, 0.5);

    CHECK(between.size() == 2);
    CHECK(between.keys()[0] == X(0));
    CHECK(between.keys()[1] == X(1));
    CHECK(between.measured() == 5.0);
    CHECK(between.sigma() == 0.5);
}

TEST_CASE("BetweenFactor - zero error at measurement") {
    BetweenFactor between(X(0), X(1), 5.0, 1.0);

    Values values;
    values.insert(X(0), 2.0);
    values.insert(X(1), 7.0); // Difference = 5.0 (exact)

    double err = between.error(values);
    CHECK(err == doctest::Approx(0.0));
}

TEST_CASE("BetweenFactor - error with offset") {
    BetweenFactor between(X(0), X(1), 5.0, 1.0);

    Values values;
    values.insert(X(0), 2.0);
    values.insert(X(1), 8.0); // Difference = 6.0 (off by 1)

    // Residual = (6 - 5) / 1 = 1
    // Error = 0.5 * 1^2 = 0.5
    double err = between.error(values);
    CHECK(err == doctest::Approx(0.5));
}

TEST_CASE("BetweenFactor - error scales with sigma") {
    BetweenFactor between1(X(0), X(1), 5.0, 1.0);
    BetweenFactor between2(X(0), X(1), 5.0, 2.0);

    Values values;
    values.insert(X(0), 0.0);
    values.insert(X(1), 7.0); // Difference = 7.0 (off by 2)

    double err1 = between1.error(values);
    double err2 = between2.error(values);

    // Error1 = 0.5 * (2/1)^2 = 2.0
    // Error2 = 0.5 * (2/2)^2 = 0.5
    CHECK(err1 == doctest::Approx(2.0));
    CHECK(err2 == doctest::Approx(0.5));
}

TEST_CASE("BetweenFactor - works with negative differences") {
    BetweenFactor between(X(0), X(1), -3.0, 1.0);

    Values values;
    values.insert(X(0), 5.0);
    values.insert(X(1), 2.0); // Difference = -3.0 (exact)

    double err = between.error(values);
    CHECK(err == doctest::Approx(0.0));
}

TEST_CASE("BetweenFactor - negative sigma throws") {
    CHECK_THROWS_AS(BetweenFactor(X(0), X(1), 0.0, -1.0), std::invalid_argument);
}

TEST_CASE("BetweenFactor - zero sigma throws") {
    CHECK_THROWS_AS(BetweenFactor(X(0), X(1), 0.0, 0.0), std::invalid_argument);
}

TEST_CASE("NonlinearFactorGraph - can hold NonlinearFactors") {
    Graph<NonlinearFactor> graph;

    graph.add(std::make_shared<PriorFactor>(X(0), 0.0, 1.0));
    graph.add(std::make_shared<BetweenFactor>(X(0), X(1), 1.0, 0.5));

    CHECK(graph.size() == 2);
}

TEST_CASE("NonlinearFactorGraph - compute total error") {
    Graph<NonlinearFactor> graph;

    graph.add(std::make_shared<PriorFactor>(X(0), 0.0, 1.0));
    graph.add(std::make_shared<BetweenFactor>(X(0), X(1), 1.0, 1.0));

    Values values;
    values.insert(X(0), 0.0); // Exact for prior
    values.insert(X(1), 1.0); // Exact for between

    // Compute total error
    double total_error = 0.0;
    for (const auto &factor : graph) {
        total_error += factor->error(values);
    }

    CHECK(total_error == doctest::Approx(0.0));
}

TEST_CASE("NonlinearFactorGraph - total error with offsets") {
    Graph<NonlinearFactor> graph;

    // Prior: x0 = 0 with sigma=1
    graph.add(std::make_shared<PriorFactor>(X(0), 0.0, 1.0));
    // Between: x1 - x0 = 1 with sigma=1
    graph.add(std::make_shared<BetweenFactor>(X(0), X(1), 1.0, 1.0));

    Values values;
    values.insert(X(0), 1.0); // Off by 1 from prior
    values.insert(X(1), 2.0); // Exact difference

    double total_error = 0.0;
    for (const auto &factor : graph) {
        total_error += factor->error(values);
    }

    // Prior error = 0.5 * (1-0)^2 = 0.5
    // Between error = 0.5 * ((2-1) - 1)^2 = 0
    // Total = 0.5
    CHECK(total_error == doctest::Approx(0.5));
}
