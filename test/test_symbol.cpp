#include "graphix/kernel.hpp"
#include <doctest/doctest.h>
#include <unordered_map>

TEST_CASE("Symbol construction and encoding") {
    graphix::Symbol s1('x', 0);
    graphix::Symbol s2('l', 5);
    graphix::Symbol s3('p', 100);

    CHECK(s1.chr() == 'x');
    CHECK(s1.index() == 0);

    CHECK(s2.chr() == 'l');
    CHECK(s2.index() == 5);

    CHECK(s3.chr() == 'p');
    CHECK(s3.index() == 100);
}

TEST_CASE("Symbol helper functions") {
    auto x0 = graphix::X(0);
    auto x5 = graphix::X(5);
    auto l10 = graphix::L(10);
    auto p99 = graphix::P(99);

    CHECK(x0.chr() == 'x');
    CHECK(x0.index() == 0);

    CHECK(x5.chr() == 'x');
    CHECK(x5.index() == 5);

    CHECK(l10.chr() == 'l');
    CHECK(l10.index() == 10);

    CHECK(p99.chr() == 'p');
    CHECK(p99.index() == 99);
}

TEST_CASE("Symbol comparison") {
    auto x0 = graphix::X(0);
    auto x1 = graphix::X(1);
    auto x0_copy = graphix::X(0);

    CHECK(x0 == x0_copy);
    CHECK(x0 != x1);
}

TEST_CASE("Symbol to Key conversion") {
    auto x5 = graphix::X(5);
    graphix::Key k = x5.key();

    CHECK(k == x5);

    graphix::Key k2 = x5;
    CHECK(k2 == k);
}

TEST_CASE("Symbol can be used in hash containers") {
    std::unordered_map<graphix::Symbol, int> symbol_map;

    symbol_map[graphix::X(0)] = 100;
    symbol_map[graphix::X(1)] = 200;
    symbol_map[graphix::L(5)] = 300;

    CHECK(symbol_map.size() == 3);
    CHECK(symbol_map[graphix::X(0)] == 100);
    CHECK(symbol_map[graphix::L(5)] == 300);
}
