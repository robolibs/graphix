#include "graphix/kernel.hpp"
#include "graphix/smallvec.hpp"
#include <doctest/doctest.h>

TEST_CASE("SmallVec default construction") {
    graphix::SmallVec<int, 4> vec;
    CHECK(vec.empty());
    CHECK(vec.size() == 0);
}

TEST_CASE("SmallVec initializer list") {
    graphix::SmallVec<int, 4> vec{1, 2, 3};
    CHECK(vec.size() == 3);
    CHECK(vec[0] == 1);
    CHECK(vec[1] == 2);
    CHECK(vec[2] == 3);
}

TEST_CASE("SmallVec push_back inline storage") {
    graphix::SmallVec<int, 4> vec;

    vec.push_back(10);
    vec.push_back(20);
    vec.push_back(30);

    CHECK(vec.size() == 3);
    CHECK(vec[0] == 10);
    CHECK(vec[1] == 20);
    CHECK(vec[2] == 30);
}

TEST_CASE("SmallVec transition to heap") {
    graphix::SmallVec<int, 4> vec;

    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);
    vec.push_back(4);
    CHECK(vec.size() == 4);

    vec.push_back(5);
    vec.push_back(6);

    CHECK(vec.size() == 6);
    CHECK(vec[0] == 1);
    CHECK(vec[4] == 5);
    CHECK(vec[5] == 6);
}

TEST_CASE("SmallVec copy construction") {
    graphix::SmallVec<int, 4> vec1{1, 2, 3};
    graphix::SmallVec<int, 4> vec2 = vec1;

    CHECK(vec2.size() == 3);
    CHECK(vec2[0] == 1);
    CHECK(vec2[1] == 2);
    CHECK(vec2[2] == 3);
}

TEST_CASE("SmallVec copy assignment") {
    graphix::SmallVec<int, 4> vec1{1, 2, 3};
    graphix::SmallVec<int, 4> vec2;

    vec2 = vec1;

    CHECK(vec2.size() == 3);
    CHECK(vec2[0] == 1);
}

TEST_CASE("SmallVec clear") {
    graphix::SmallVec<int, 4> vec{1, 2, 3};
    CHECK(vec.size() == 3);

    vec.clear();
    CHECK(vec.empty());
    CHECK(vec.size() == 0);
}

TEST_CASE("SmallVec iteration") {
    graphix::SmallVec<int, 4> vec{10, 20, 30};

    int sum = 0;
    for (auto val : vec) {
        sum += val;
    }

    CHECK(sum == 60);
}

TEST_CASE("SmallVec with Keys") {
    graphix::SmallVec<graphix::Key, 6> keys;

    keys.push_back(graphix::X(0));
    keys.push_back(graphix::X(1));
    keys.push_back(graphix::L(5));

    CHECK(keys.size() == 3);

    int count = 0;
    for (auto k : keys) {
        count++;
    }
    CHECK(count == 3);
}
