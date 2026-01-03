#include "graphix/kernel.hpp"
#include <doctest/doctest.h>
#include <unordered_map>

struct TestData {
    int value;
    TestData(int v = 0) : value(v) {}
};

TEST_CASE("Id default construction") {
    graphix::Id<TestData> id;
    CHECK_FALSE(id.is_valid());
    CHECK(id.value() == graphix::Id<TestData>::invalid_id());
}

TEST_CASE("Id explicit construction") {
    graphix::Id<TestData> id(42);
    CHECK(id.is_valid());
    CHECK(id.value() == 42);
}

TEST_CASE("Id comparison") {
    graphix::Id<TestData> id1(10);
    graphix::Id<TestData> id2(20);
    graphix::Id<TestData> id3(10);

    CHECK(id1 == id3);
    CHECK(id1 != id2);
    CHECK(id1 < id2);
}

TEST_CASE("Id can be used in hash containers") {
    std::unordered_map<graphix::Id<TestData>, int> id_map;

    graphix::Id<TestData> id1(1);
    graphix::Id<TestData> id2(2);

    id_map[id1] = 100;
    id_map[id2] = 200;

    CHECK(id_map.size() == 2);
    CHECK(id_map[id1] == 100);
}
