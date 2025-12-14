#include "graphix/factor/factor.hpp"
#include "graphix/factor/graph.hpp"
#include "graphix/kernel.hpp"
#include <doctest/doctest.h>

using namespace graphix;
using namespace graphix::factor;

TEST_CASE("FactorGraph - default constructor") {
    Graph<> graph;

    CHECK(graph.size() == 0);
    CHECK(graph.empty());
}

TEST_CASE("FactorGraph - add single factor") {
    Graph<> graph;
    auto f = std::make_shared<Factor>(std::initializer_list<Key>{1, 2});

    graph.add(f);

    CHECK(graph.size() == 1);
    CHECK_FALSE(graph.empty());
}

TEST_CASE("FactorGraph - add multiple factors") {
    Graph<> graph;

    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{1, 2}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{2, 3}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{3, 4}));

    CHECK(graph.size() == 3);
}

TEST_CASE("FactorGraph - access by index") {
    Graph<> graph;

    auto f1 = std::make_shared<Factor>(std::initializer_list<Key>{1, 2});
    auto f2 = std::make_shared<Factor>(std::initializer_list<Key>{3, 4});

    graph.add(f1);
    graph.add(f2);

    CHECK(graph[0] == f1);
    CHECK(graph[1] == f2);
}

TEST_CASE("FactorGraph - at() method") {
    Graph<> graph;

    auto f = std::make_shared<Factor>(std::initializer_list<Key>{1, 2, 3});
    graph.add(f);

    CHECK(graph.at(0) == f);
    CHECK_THROWS_AS(graph.at(10), std::out_of_range);
}

TEST_CASE("FactorGraph - keys() returns all unique variables") {
    Graph<> graph;

    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{1, 2}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{2, 3}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{3, 4}));

    auto keys = graph.keys();

    CHECK(keys.size() == 4);
    CHECK(keys.count(1) == 1);
    CHECK(keys.count(2) == 1);
    CHECK(keys.count(3) == 1);
    CHECK(keys.count(4) == 1);
}

TEST_CASE("FactorGraph - keys() with overlapping factors") {
    Graph<> graph;

    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{1, 2, 3}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{2, 3, 4}));

    auto keys = graph.keys();

    CHECK(keys.size() == 4); // Unique keys only
}

TEST_CASE("FactorGraph - range-based for loop") {
    Graph<> graph;

    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{1, 2}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{3, 4}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{5, 6}));

    size_t count = 0;
    for (const auto &factor : graph) {
        count++;
        CHECK(factor != nullptr);
    }

    CHECK(count == 3);
}

TEST_CASE("FactorGraph - clear") {
    Graph<> graph;

    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{1, 2}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{3, 4}));

    CHECK(graph.size() == 2);

    graph.clear();

    CHECK(graph.size() == 0);
    CHECK(graph.empty());
}

TEST_CASE("FactorGraph - nullptr factors are handled") {
    Graph<> graph;

    graph.add(nullptr); // Should not crash

    CHECK(graph.size() == 0); // nullptr not added
}

TEST_CASE("FactorGraph - with Symbol keys") {
    Graph<> graph;

    Key x0 = X(0);
    Key x1 = X(1);
    Key l0 = L(0);

    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{x0, x1}));
    graph.add(std::make_shared<Factor>(std::initializer_list<Key>{x1, l0}));

    auto keys = graph.keys();

    CHECK(keys.size() == 3);
    CHECK(keys.count(x0) == 1);
    CHECK(keys.count(x1) == 1);
    CHECK(keys.count(l0) == 1);
}

TEST_CASE("FactorGraph - empty graph has no keys") {
    Graph<> graph;

    auto keys = graph.keys();

    CHECK(keys.empty());
}

TEST_CASE("FactorGraph - large graph") {
    Graph<> graph;

    for (size_t i = 0; i < 100; ++i) {
        graph.add(std::make_shared<Factor>(std::initializer_list<Key>{i, i + 1}));
    }

    CHECK(graph.size() == 100);

    auto keys = graph.keys();
    CHECK(keys.size() == 101); // 0 through 100
}
