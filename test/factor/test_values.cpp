#include "graphix/factor/values.hpp"
#include "graphix/kernel.hpp"
#include <doctest/doctest.h>

using namespace graphix;
using namespace graphix::factor;

TEST_CASE("Values - basic insert and retrieve double") {
    Values values;
    Key k1 = 1;

    values.insert(k1, 5.0);

    CHECK(values.at<double>(k1) == 5.0);
    CHECK(values.size() == 1);
    CHECK_FALSE(values.empty());
}

TEST_CASE("Values - multiple inserts") {
    Values values;

    values.insert(1, 1.0);
    values.insert(2, 2.5);
    values.insert(3, -3.7);

    CHECK(values.size() == 3);
    CHECK(values.at<double>(1) == 1.0);
    CHECK(values.at<double>(2) == 2.5);
    CHECK(values.at<double>(3) == -3.7);
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

    CHECK_THROWS_AS(values.at<double>(999), std::out_of_range);
}

TEST_CASE("Values - insert throws on duplicate key") {
    Values values;

    values.insert(5, 10.0);

    CHECK_THROWS_AS(values.insert(5, 20.0), std::runtime_error);
    CHECK(values.at<double>(5) == 10.0); // Original value unchanged
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
    CHECK(copy.at<double>(1) == 10.0);
    CHECK(copy.at<double>(2) == 20.0);

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
    CHECK(assigned.at<double>(1) == 100.0);
    CHECK(assigned.at<double>(2) == 200.0);
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
    CHECK(values.at<double>(x0) == 1.0);
    CHECK(values.at<double>(x1) == 2.0);
    CHECK(values.at<double>(l0) == 3.0);
}

// ============================================================================
// Type erasure tests - different types
// ============================================================================

TEST_CASE("Values - store and retrieve int") {
    Values values;

    values.insert<int>(1, 42);
    values.insert<int>(2, -17);

    CHECK(values.at<int>(1) == 42);
    CHECK(values.at<int>(2) == -17);
}

TEST_CASE("Values - store and retrieve std::vector<double>") {
    Values values;

    std::vector<double> vec1 = {1.0, 2.0, 3.0};
    std::vector<double> vec2 = {4.5, 5.5};

    values.insert(1, vec1);
    values.insert(2, vec2);

    auto retrieved1 = values.at<std::vector<double>>(1);
    auto retrieved2 = values.at<std::vector<double>>(2);

    CHECK(retrieved1.size() == 3);
    CHECK(retrieved1[0] == 1.0);
    CHECK(retrieved1[2] == 3.0);
    CHECK(retrieved2.size() == 2);
    CHECK(retrieved2[1] == 5.5);
}

TEST_CASE("Values - store and retrieve std::string") {
    Values values;

    values.insert<std::string>(1, "hello");
    values.insert<std::string>(2, "world");

    CHECK(values.at<std::string>(1) == "hello");
    CHECK(values.at<std::string>(2) == "world");
}

struct TestStruct {
    int a;
    double b;

    bool operator==(const TestStruct &other) const { return a == other.a && b == other.b; }
};

TEST_CASE("Values - store and retrieve custom struct") {
    Values values;

    TestStruct s1{10, 3.14};
    TestStruct s2{20, 2.71};

    values.insert(1, s1);
    values.insert(2, s2);

    auto r1 = values.at<TestStruct>(1);
    auto r2 = values.at<TestStruct>(2);

    CHECK(r1.a == 10);
    CHECK(r1.b == 3.14);
    CHECK(r2.a == 20);
    CHECK(r2.b == 2.71);
}

TEST_CASE("Values - type mismatch throws") {
    Values values;

    values.insert<int>(1, 42);
    values.insert<double>(2, 3.14);

    CHECK_THROWS_AS(values.at<double>(1), std::runtime_error);
    CHECK_THROWS_AS(values.at<int>(2), std::runtime_error);
}

TEST_CASE("Values - mixed types in same container") {
    Values values;

    values.insert<int>(1, 100);
    values.insert<double>(2, 2.5);
    values.insert<std::string>(3, "test");

    CHECK(values.size() == 3);
    CHECK(values.at<int>(1) == 100);
    CHECK(values.at<double>(2) == 2.5);
    CHECK(values.at<std::string>(3) == "test");
}

TEST_CASE("Values - copy with mixed types") {
    Values original;

    original.insert<int>(1, 50);
    original.insert<std::string>(2, "copy");
    original.insert<double>(3, 9.99);

    Values copy(original);

    CHECK(copy.size() == 3);
    CHECK(copy.at<int>(1) == 50);
    CHECK(copy.at<std::string>(2) == "copy");
    CHECK(copy.at<double>(3) == 9.99);
}

TEST_CASE("Values - erase works with any type") {
    Values values;

    values.insert<int>(1, 10);
    values.insert<std::string>(2, "will be erased");
    values.insert<double>(3, 3.0);

    values.erase(2);

    CHECK(values.size() == 2);
    CHECK(values.exists(1));
    CHECK_FALSE(values.exists(2));
    CHECK(values.exists(3));
}
