#include "doctest.h"
#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/between_factor.hpp"
#include "graphix/factor/nonlinear/gradient_descent.hpp"
#include "graphix/factor/nonlinear/prior_factor.hpp"
#include "graphix/kernel.hpp"

using namespace graphix;
using namespace graphix::factor;

TEST_CASE("GradientDescent - default constructor") {
    GradientDescentOptimizer optimizer;

    auto params = optimizer.parameters();
    CHECK(params.max_iterations == 100);
    CHECK(params.step_size == doctest::Approx(0.01));
    CHECK(params.tolerance == doctest::Approx(1e-6));
    CHECK(params.h == doctest::Approx(1e-5));
    CHECK(params.verbose == false);
}

TEST_CASE("GradientDescent - custom parameters") {
    GradientDescentOptimizer::Parameters params;
    params.max_iterations = 50;
    params.step_size = 0.05;
    params.tolerance = 1e-4;
    params.h = 1e-6;
    params.verbose = true;

    GradientDescentOptimizer optimizer(params);

    auto stored_params = optimizer.parameters();
    CHECK(stored_params.max_iterations == 50);
    CHECK(stored_params.step_size == doctest::Approx(0.05));
    CHECK(stored_params.tolerance == doctest::Approx(1e-4));
    CHECK(stored_params.h == doctest::Approx(1e-6));
    CHECK(stored_params.verbose == true);
}

TEST_CASE("GradientDescent - single prior factor") {
    // Create a simple graph: x0 should be 5.0
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 5.0, 0.1));

    // Start with wrong value
    Values initial;
    initial.insert(X(0), 0.0);

    // Optimize
    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.1;
    params.max_iterations = 100;
    params.tolerance = 1e-6;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    // Should converge to 5.0
    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(5.0).epsilon(0.01));
    CHECK(result.final_error == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("GradientDescent - prior factor with different initial guess") {
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 10.0, 0.5));

    Values initial;
    initial.insert(X(0), 100.0); // Very far from prior

    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.1;
    params.max_iterations = 200;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(10.0).epsilon(0.01));
}

TEST_CASE("GradientDescent - chain with between factors") {
    // Create a simple 1D SLAM chain:
    // x0 = 0 (prior)
    // x1 = x0 + 1 (between)
    // x2 = x1 + 1 (between)
    // Expected: x0=0, x1=1, x2=2

    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 0.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(0), X(1), 1.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(1), X(2), 1.0, 0.1));

    // Start with wrong guesses
    Values initial;
    initial.insert(X(0), 0.5);
    initial.insert(X(1), 0.5);
    initial.insert(X(2), 0.5);

    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.05;
    params.max_iterations = 500;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(0.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(1)) == doctest::Approx(1.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(2)) == doctest::Approx(2.0).epsilon(0.01));
}

TEST_CASE("GradientDescent - longer chain") {
    // Longer chain: x0=0, x1=1, x2=2, x3=3, x4=4
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 0.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(0), X(1), 1.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(1), X(2), 1.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(2), X(3), 1.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(3), X(4), 1.0, 0.1));

    Values initial;
    initial.insert(X(0), 10.0);
    initial.insert(X(1), 10.0);
    initial.insert(X(2), 10.0);
    initial.insert(X(3), 10.0);
    initial.insert(X(4), 10.0);

    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.05;
    params.max_iterations = 1000;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(0.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(1)) == doctest::Approx(1.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(2)) == doctest::Approx(2.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(3)) == doctest::Approx(3.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(4)) == doctest::Approx(4.0).epsilon(0.01));
}

TEST_CASE("GradientDescent - loop closure") {
    // Create a small loop:
    // x0 = 0 (prior)
    // x1 = x0 + 1
    // x2 = x1 + 1
    // x0 = x2 - 2 (loop closure)

    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 0.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(0), X(1), 1.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(1), X(2), 1.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(2), X(0), -2.0, 0.1)); // Loop closure

    Values initial;
    initial.insert(X(0), 0.1);
    initial.insert(X(1), 0.9);
    initial.insert(X(2), 2.1);

    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.05;
    params.max_iterations = 500;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(0.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(1)) == doctest::Approx(1.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(2)) == doctest::Approx(2.0).epsilon(0.01));
}

TEST_CASE("GradientDescent - multiple independent variables") {
    // Two independent priors
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 5.0, 0.1));
    graph.add(std::make_shared<PriorFactor>(X(1), 10.0, 0.1));

    Values initial;
    initial.insert(X(0), 0.0);
    initial.insert(X(1), 0.0);

    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.1;
    params.max_iterations = 200;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(5.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(1)) == doctest::Approx(10.0).epsilon(0.01));
}

TEST_CASE("GradientDescent - error decreases") {
    // Verify that error actually decreases during optimization
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 5.0, 0.1));

    Values initial;
    initial.insert(X(0), 0.0);

    // Compute initial error
    double initial_error = 0.0;
    for (const auto &factor : graph) {
        initial_error += factor->error(initial);
    }

    GradientDescentOptimizer optimizer;
    auto result = optimizer.optimize(graph, initial);

    // Final error should be much smaller
    CHECK(result.final_error < initial_error);
    CHECK(result.final_error == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("GradientDescent - max iterations limit") {
    // Set very low max iterations to test early stopping
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 100.0, 0.1));

    Values initial;
    initial.insert(X(0), 0.0);

    GradientDescentOptimizer::Parameters params;
    params.max_iterations = 5; // Very few iterations
    params.step_size = 0.01;   // Small step size

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    // Should hit max iterations
    CHECK(result.iterations <= 5);
    // May not have converged
    // But error should have improved
    CHECK(result.values.at<double>(X(0)) > 0.0); // Should have moved towards 100
}

TEST_CASE("GradientDescent - already optimal") {
    // Start at optimal solution
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 5.0, 0.1));

    Values initial;
    initial.insert(X(0), 5.0); // Already at optimum

    GradientDescentOptimizer optimizer;
    auto result = optimizer.optimize(graph, initial);

    // Should converge immediately (or after 1 iteration)
    CHECK(result.iterations <= 1);
    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(5.0).epsilon(0.001));
    CHECK(result.final_error == doctest::Approx(0.0).epsilon(0.001));
}

TEST_CASE("GradientDescent - result contains all info") {
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 3.0, 0.1));

    Values initial;
    initial.insert(X(0), 0.0);

    GradientDescentOptimizer optimizer;
    auto result = optimizer.optimize(graph, initial);

    // Check result has all fields
    CHECK(result.values.exists(X(0)));
    CHECK(result.final_error >= 0.0);
    CHECK(result.iterations >= 0);
    CHECK(result.gradient_norm >= 0.0);
    // converged is bool, either true or false
}

TEST_CASE("GradientDescent - overdetermined system") {
    // More constraints than variables
    // x0 should be 5, but also between x0 and nothing...
    // Actually, let's do: x0 = 5 (prior1), x0 = 6 (prior2)
    // Optimal is x0 = 5.5 (compromise)

    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 5.0, 0.1));
    graph.add(std::make_shared<PriorFactor>(X(0), 6.0, 0.1));

    Values initial;
    initial.insert(X(0), 0.0);

    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.1;
    params.max_iterations = 200;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    // Should converge to average
    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(5.5).epsilon(0.01));
}

TEST_CASE("GradientDescent - weighted by sigma") {
    // Two priors with different sigmas (uncertainties)
    // x0 = 5 with sigma=0.1 (very certain)
    // x0 = 10 with sigma=1.0 (less certain)
    // Should be closer to 5 than 10

    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), 5.0, 0.1));  // Strong prior
    graph.add(std::make_shared<PriorFactor>(X(0), 10.0, 1.0)); // Weak prior

    Values initial;
    initial.insert(X(0), 0.0);

    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.1;
    params.max_iterations = 300;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    // Should be closer to 5 than to 10 due to weighting
    double x = result.values.at<double>(X(0));
    CHECK(x > 5.0);
    CHECK(x < 7.0); // Much closer to 5 than 10
}

TEST_CASE("GradientDescent - negative values") {
    // Test that negative values work fine
    Graph<NonlinearFactor> graph;
    graph.add(std::make_shared<PriorFactor>(X(0), -5.0, 0.1));
    graph.add(std::make_shared<BetweenFactor>(X(0), X(1), -2.0, 0.1));

    Values initial;
    initial.insert(X(0), 0.0);
    initial.insert(X(1), 0.0);

    GradientDescentOptimizer::Parameters params;
    params.step_size = 0.05;
    params.max_iterations = 500;

    GradientDescentOptimizer optimizer(params);
    auto result = optimizer.optimize(graph, initial);

    CHECK(result.converged == true);
    CHECK(result.values.at<double>(X(0)) == doctest::Approx(-5.0).epsilon(0.01));
    CHECK(result.values.at<double>(X(1)) == doctest::Approx(-7.0).epsilon(0.01));
}

TEST_CASE("GradientDescent - set_parameters") {
    GradientDescentOptimizer optimizer;

    GradientDescentOptimizer::Parameters new_params;
    new_params.max_iterations = 999;
    new_params.step_size = 0.123;

    optimizer.set_parameters(new_params);

    auto params = optimizer.parameters();
    CHECK(params.max_iterations == 999);
    CHECK(params.step_size == doctest::Approx(0.123));
}
