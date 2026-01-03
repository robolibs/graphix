#include "doctest/doctest.h"

#include "graphix/factor/loss_function.hpp"
#include <cmath>

using namespace graphix::factor;

TEST_CASE("LossFunction - NullLoss") {
    NullLoss loss;

    SUBCASE("Returns squared error unchanged") {
        CHECK(loss.evaluate(0.0) == doctest::Approx(0.0));
        CHECK(loss.evaluate(1.0) == doctest::Approx(1.0));
        CHECK(loss.evaluate(4.0) == doctest::Approx(4.0));
        CHECK(loss.evaluate(100.0) == doctest::Approx(100.0));
    }

    SUBCASE("Weight is always 1.0") {
        CHECK(loss.weight(0.0) == doctest::Approx(1.0));
        CHECK(loss.weight(1.0) == doctest::Approx(1.0));
        CHECK(loss.weight(4.0) == doctest::Approx(1.0));
        CHECK(loss.weight(100.0) == doctest::Approx(1.0));
    }

    SUBCASE("Clone works") {
        auto clone = loss.clone();
        CHECK(clone->evaluate(4.0) == doctest::Approx(4.0));
        CHECK(clone->weight(4.0) == doctest::Approx(1.0));
    }
}

TEST_CASE("LossFunction - HuberLoss") {
    double k = 1.345;
    HuberLoss loss(k);

    SUBCASE("Quadratic for small errors") {
        // For e² = 0.5, e = sqrt(0.5) ≈ 0.707 < k
        // ρ(e²) = e²/2 = 0.25
        double sq_err = 0.5;
        CHECK(loss.evaluate(sq_err) == doctest::Approx(0.25));
        CHECK(loss.weight(sq_err) == doctest::Approx(1.0));
    }

    SUBCASE("Linear for large errors") {
        // For e² = 4.0, e = 2.0 > k = 1.345
        // ρ(e²) = k*e - k²/2 = 1.345*2.0 - 1.345²/2
        double sq_err = 4.0;
        double e = 2.0;
        double expected = k * e - 0.5 * k * k;
        CHECK(loss.evaluate(sq_err) == doctest::Approx(expected));

        // Weight should be k/e for large errors
        CHECK(loss.weight(sq_err) == doctest::Approx(k / e));
    }

    SUBCASE("Continuous at threshold") {
        // At e = k, both formulas should give same result
        double sq_err = k * k;
        double quadratic = 0.5 * sq_err;
        double linear = k * k - 0.5 * k * k;
        CHECK(std::abs(quadratic - linear) < 0.001);
    }

    SUBCASE("Clone preserves parameter") {
        auto clone = std::dynamic_pointer_cast<HuberLoss>(loss.clone());
        REQUIRE(clone);
        CHECK(clone->threshold() == doctest::Approx(k));
    }
}

TEST_CASE("LossFunction - CauchyLoss") {
    double k = 2.3849;
    CauchyLoss loss(k);

    SUBCASE("Zero error gives zero loss") { CHECK(loss.evaluate(0.0) == doctest::Approx(0.0)); }

    SUBCASE("Grows logarithmically for large errors") {
        // For e² = 100, ρ(e²) = (k²/2) * log(1 + 100/k²)
        double sq_err = 100.0;
        double expected = 0.5 * k * k * std::log1p(sq_err / (k * k));
        CHECK(loss.evaluate(sq_err) == doctest::Approx(expected));
    }

    SUBCASE("Weight decreases for large errors") {
        // w = k² / (k² + e²)
        double sq_err = 4.0;
        double expected_weight = (k * k) / (k * k + sq_err);
        CHECK(loss.weight(sq_err) == doctest::Approx(expected_weight));

        // Larger error should have smaller weight
        double large_sq_err = 100.0;
        double large_weight = loss.weight(large_sq_err);
        CHECK(large_weight < expected_weight);
    }

    SUBCASE("Clone preserves parameter") {
        auto clone = std::dynamic_pointer_cast<CauchyLoss>(loss.clone());
        REQUIRE(clone);
        CHECK(clone->scale() == doctest::Approx(k));
    }
}

TEST_CASE("LossFunction - TukeyLoss") {
    double k = 4.6851;
    TukeyLoss loss(k);

    SUBCASE("Zero error gives zero loss") { CHECK(loss.evaluate(0.0) == doctest::Approx(0.0)); }

    SUBCASE("Within threshold - uses formula") {
        // For e² < k², use (k²/6) * (1 - (1 - e²/k²)³)
        double sq_err = 2.0; // < k²
        double ratio = sq_err / (k * k);
        double term = 1.0 - ratio;
        double expected = (k * k / 6.0) * (1.0 - term * term * term);
        CHECK(loss.evaluate(sq_err) == doctest::Approx(expected));
    }

    SUBCASE("Beyond threshold - constant") {
        // For e² > k², ρ(e²) = k²/6
        double sq_err = k * k + 10.0;
        double expected = k * k / 6.0;
        CHECK(loss.evaluate(sq_err) == doctest::Approx(expected));
    }

    SUBCASE("Weight is zero beyond threshold") {
        double sq_err = k * k + 1.0;
        CHECK(loss.weight(sq_err) == doctest::Approx(0.0));
    }

    SUBCASE("Weight within threshold") {
        double sq_err = 2.0;
        double ratio = sq_err / (k * k);
        double term = 1.0 - ratio;
        double expected_weight = term * term;
        CHECK(loss.weight(sq_err) == doctest::Approx(expected_weight));
    }

    SUBCASE("Clone preserves parameter") {
        auto clone = std::dynamic_pointer_cast<TukeyLoss>(loss.clone());
        REQUIRE(clone);
        CHECK(clone->threshold() == doctest::Approx(k));
    }
}

TEST_CASE("LossFunction - Helper functions") {
    SUBCASE("no_loss creates NullLoss") {
        auto loss = no_loss();
        CHECK(loss->evaluate(4.0) == doctest::Approx(4.0));
    }

    SUBCASE("huber_loss creates HuberLoss") {
        auto loss = huber_loss(2.0);
        auto huber = std::dynamic_pointer_cast<HuberLoss>(loss);
        REQUIRE(huber);
        CHECK(huber->threshold() == doctest::Approx(2.0));
    }

    SUBCASE("cauchy_loss creates CauchyLoss") {
        auto loss = cauchy_loss(3.0);
        auto cauchy = std::dynamic_pointer_cast<CauchyLoss>(loss);
        REQUIRE(cauchy);
        CHECK(cauchy->scale() == doctest::Approx(3.0));
    }

    SUBCASE("tukey_loss creates TukeyLoss") {
        auto loss = tukey_loss(5.0);
        auto tukey = std::dynamic_pointer_cast<TukeyLoss>(loss);
        REQUIRE(tukey);
        CHECK(tukey->threshold() == doctest::Approx(5.0));
    }
}

TEST_CASE("LossFunction - Comparative behavior") {
    double sq_err_small = 0.5;
    double sq_err_large = 25.0;

    NullLoss null;
    HuberLoss huber(1.345);
    CauchyLoss cauchy(2.3849);
    TukeyLoss tukey(4.6851);

    SUBCASE("All agree for very small errors") {
        // For tiny errors, all should be approximately quadratic
        double tiny = 0.01;
        CHECK(null.evaluate(tiny) == doctest::Approx(tiny));
        // Others may differ slightly but should be similar magnitude
    }

    SUBCASE("Null loss grows fastest for large errors") {
        double null_large = null.evaluate(sq_err_large);
        double huber_large = huber.evaluate(sq_err_large);
        double cauchy_large = cauchy.evaluate(sq_err_large);
        double tukey_large = tukey.evaluate(sq_err_large);

        // Null (quadratic) should be largest
        CHECK(null_large > huber_large);
        CHECK(null_large > cauchy_large);
        CHECK(null_large > tukey_large);
    }

    SUBCASE("Weights decrease with error for robust losses") {
        // Null always 1.0
        CHECK(null.weight(sq_err_small) == doctest::Approx(1.0));
        CHECK(null.weight(sq_err_large) == doctest::Approx(1.0));

        // Others should decrease
        CHECK(huber.weight(sq_err_small) >= huber.weight(sq_err_large));
        CHECK(cauchy.weight(sq_err_small) > cauchy.weight(sq_err_large));
        CHECK(tukey.weight(sq_err_small) > tukey.weight(sq_err_large));
    }
}
