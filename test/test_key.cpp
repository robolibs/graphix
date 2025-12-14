#include "graphix/kernel.hpp"
#include <doctest/doctest.h>
#include <unordered_set>

TEST_CASE("Key basic usage") {
    graphix::Key k1 = 0;
    graphix::Key k2 = 42;
    graphix::Key k3 = 12345678901234ULL;

    CHECK(k1 == 0);
    CHECK(k2 == 42);
    CHECK(k3 == 12345678901234ULL);
    CHECK(k1 != k2);
}

TEST_CASE("Key can be used in hash containers") {
    std::unordered_set<graphix::Key> key_set;
    key_set.insert(1);
    key_set.insert(2);
    key_set.insert(3);

    CHECK(key_set.size() == 3);
    CHECK(key_set.count(1) == 1);
    CHECK(key_set.count(99) == 0);
}
