#include "graphix/factor/values.hpp"
#include "graphix/kernel.hpp"
#include <doctest/doctest.h>

using namespace graphix;
using namespace graphix::factor;

TEST_CASE("Values - basic insert and retrieve double") {
    Values values;
    Key k1 = 1;

    values.insert(k1, 5.0);

    CHECK(values.at(k1) == 5.0);
    CHECK(values.size() == 1);
    CHECK_FALSE(values.empty());
}

TEST_CASE("Values - multiple inserts") {
    Values values;

    values.insert(1, 1.0);
    values.insert(2, 2.5);
    values.insert(3, -3.7);

    CHECK(values.size() == 3);
    CHECK(values.at(1) == 1.0);
    CHECK(values.at(2) == 2.5);
    CHECK(values.at(3) == -3.7);
}

TEST_CASE("Values - exists check") {
    Values values;

    values.insert(10, 42.0);

    CHECK(values.exists(10));
    CHECK_FALSE(values.exists(99));
    CHECK_FALSE(values.exists(0));
}

TEST_CASE("Values - at() throws on non-existent key") {
    Values values;

    values.insert(1, 5.0);

    CHECK_THROWS_AS(values.at(999), std::out_of_range);
}

TEST_CASE("Values - insert throws on duplicate key") {
    Values values;

    values.insert(5, 10.0);

    CHECK_THROWS_AS(values.insert(5, 20.0), std::runtime_error);
    CHECK(values.at(5) == 10.0); // Original value unchanged
}

TEST_CASE("Values - erase key") {
    Values values;

    values.insert(1, 1.0);
    values.insert(2, 2.0);
    values.insert(3, 3.0);

    CHECK(values.size() == 3);

    values.erase(2);

    CHECK(values.size() == 2);
    CHECK(values.exists(1));
    CHECK_FALSE(values.exists(2));
    CHECK(values.exists(3));
}

TEST_CASE("Values - erase non-existent key is safe") {
    Values values;

    values.insert(1, 1.0);
    values.erase(999); // Should not throw

    CHECK(values.size() == 1);
}

TEST_CASE("Values - clear all values") {
    Values values;

    values.insert(1, 1.0);
    values.insert(2, 2.0);
    values.insert(3, 3.0);

    CHECK(values.size() == 3);

    values.clear();

    CHECK(values.size() == 0);
    CHECK(values.empty());
    CHECK_FALSE(values.exists(1));
}

TEST_CASE("Values - copy constructor") {
    Values original;

    original.insert(1, 10.0);
    original.insert(2, 20.0);

    Values copy(original);

    CHECK(copy.size() == 2);
    CHECK(copy.at(1) == 10.0);
    CHECK(copy.at(2) == 20.0);

    // Modify original - copy should be independent
    original.insert(3, 30.0);
    CHECK(original.size() == 3);
    CHECK(copy.size() == 2);
    CHECK_FALSE(copy.exists(3));
}

TEST_CASE("Values - assignment operator") {
    Values original;
    original.insert(1, 100.0);
    original.insert(2, 200.0);

    Values assigned;
    assigned.insert(99, 999.0); // Will be overwritten

    assigned = original;

    CHECK(assigned.size() == 2);
    CHECK(assigned.at(1) == 100.0);
    CHECK(assigned.at(2) == 200.0);
    CHECK_FALSE(assigned.exists(99));
}

TEST_CASE("Values - empty container") {
    Values values;

    CHECK(values.empty());
    CHECK(values.size() == 0);
    CHECK_FALSE(values.exists(1));
}

TEST_CASE("Values - Symbol keys work") {
    Values values;

    Key x0 = X(0);
    Key x1 = X(1);
    Key l0 = L(0);

    values.insert(x0, 1.0);
    values.insert(x1, 2.0);
    values.insert(l0, 3.0);

    CHECK(values.size() == 3);
    CHECK(values.at(x0) == 1.0);
    CHECK(values.at(x1) == 2.0);
    CHECK(values.at(l0) == 3.0);
}
