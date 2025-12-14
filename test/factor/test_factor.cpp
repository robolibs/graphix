#include "graphix/factor/factor.hpp"
#include "graphix/kernel.hpp"
#include <doctest/doctest.h>

using namespace graphix;
using namespace graphix::factor;

TEST_CASE("Factor - default constructor") {
    Factor f;

    CHECK(f.size() == 0);
    CHECK(f.keys().empty());
}

TEST_CASE("Factor - initializer_list constructor") {
    Factor f({1, 2, 3});

    CHECK(f.size() == 3);
    CHECK(f.keys()[0] == 1);
    CHECK(f.keys()[1] == 2);
    CHECK(f.keys()[2] == 3);
}

TEST_CASE("Factor - variadic constructor") {
    Factor f(10, 20, 30);

    CHECK(f.size() == 3);
    CHECK(f.keys()[0] == 10);
    CHECK(f.keys()[1] == 20);
    CHECK(f.keys()[2] == 30);
}

TEST_CASE("Factor - single key") {
    Factor f({42});

    CHECK(f.size() == 1);
    CHECK(f.keys()[0] == 42);
}

TEST_CASE("Factor - with Symbol keys") {
    Key x0 = X(0);
    Key x1 = X(1);
    Key l0 = L(0);

    Factor f({x0, x1, l0});

    CHECK(f.size() == 3);
    CHECK(f.keys()[0] == x0);
    CHECK(f.keys()[1] == x1);
    CHECK(f.keys()[2] == l0);
}

TEST_CASE("Factor - involves() method") {
    Factor f({1, 2, 3});

    CHECK(f.involves(1));
    CHECK(f.involves(2));
    CHECK(f.involves(3));
    CHECK_FALSE(f.involves(99));
    CHECK_FALSE(f.involves(0));
}

TEST_CASE("Factor - large factor") {
    Factor f({1, 2, 3, 4, 5, 6, 7, 8});

    CHECK(f.size() == 8);
    CHECK(f.involves(1));
    CHECK(f.involves(8));
    CHECK_FALSE(f.involves(9));
}

TEST_CASE("Factor - keys() returns const reference") {
    Factor f({1, 2, 3});

    const auto &keys = f.keys();

    CHECK(keys.size() == 3);
    CHECK(keys[0] == 1);
}

TEST_CASE("Factor - empty factor involves nothing") {
    Factor f;

    CHECK_FALSE(f.involves(1));
    CHECK_FALSE(f.involves(0));
}

TEST_CASE("Factor - duplicate keys allowed") {
    Factor f({1, 1, 2});

    CHECK(f.size() == 3);
    CHECK(f.keys()[0] == 1);
    CHECK(f.keys()[1] == 1);
    CHECK(f.keys()[2] == 2);
}
