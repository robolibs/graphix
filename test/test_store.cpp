#include "graphix/store.hpp"
#include <doctest/doctest.h>

struct TestData {
    int value;
    TestData(int v = 0) : value(v) {}
};

TEST_CASE("Store add and access") {
    graphix::Store<TestData> store;

    auto id1 = store.add(TestData(10));
    auto id2 = store.add(TestData(20));
    auto id3 = store.add(TestData(30));

    CHECK(store[id1].value == 10);
    CHECK(store[id2].value == 20);
    CHECK(store[id3].value == 30);
}

TEST_CASE("Store size tracking") {
    graphix::Store<TestData> store;
    CHECK(store.empty());
    CHECK(store.size() == 0);

    auto id1 = store.add(TestData(1));
    CHECK(store.size() == 1);
    CHECK_FALSE(store.empty());

    auto id2 = store.add(TestData(2));
    CHECK(store.size() == 2);
}

TEST_CASE("Store remove and reuse") {
    graphix::Store<TestData> store;

    auto id1 = store.add(TestData(10));
    auto id2 = store.add(TestData(20));
    auto id3 = store.add(TestData(30));

    CHECK(store.size() == 3);

    store.remove(id2);
    CHECK(store.size() == 2);
    CHECK_FALSE(store.contains(id2));

    auto id4 = store.add(TestData(40));
    CHECK(store.size() == 3);
}

TEST_CASE("Store contains check") {
    graphix::Store<TestData> store;

    auto id1 = store.add(TestData(10));
    CHECK(store.contains(id1));

    graphix::Id<TestData> invalid_id(999);
    CHECK_FALSE(store.contains(invalid_id));

    store.remove(id1);
    CHECK_FALSE(store.contains(id1));
}

TEST_CASE("Store modification") {
    graphix::Store<TestData> store;

    auto id = store.add(TestData(10));
    CHECK(store[id].value == 10);

    store[id].value = 99;
    CHECK(store[id].value == 99);
}
