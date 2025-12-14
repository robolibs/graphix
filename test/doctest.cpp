#include <doctest/doctest.h>

TEST_CASE("Basic assertions") {
    CHECK(1 + 1 == 2);
    CHECK(true);
    CHECK_FALSE(false);
    REQUIRE(42 == 42);
}

TEST_CASE("Integer operations") {
    int a = 5;
    int b = 10;
    CHECK(a + b == 15);
    CHECK(b - a == 5);
    CHECK(a * b == 50);
}

TEST_CASE("String operations") {
    std::string test = "graphix";
    CHECK(test.length() == 7);
    CHECK(test == "graphix");
    CHECK(test != "other");
}

TEST_CASE("Comparisons") {
    CHECK(10 > 5);
    CHECK(5 < 10);
    CHECK(10 >= 10);
    CHECK(5 <= 5);
}
