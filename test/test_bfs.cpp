#include "graphix/vertex/algorithms/bfs.hpp"
#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

using namespace graphix::vertex;

TEST_CASE("BFS - Simple path") {
    Graph<void> g;

    // Create simple path: 0 - 1 - 2 - 3
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v0, v1);
    g.add_edge(v1, v2);
    g.add_edge(v2, v3);

    SUBCASE("BFS from v0") {
        auto result = algorithms::bfs(g, v0);

        CHECK(result.distance.at(v0) == 0);
        CHECK(result.distance.at(v1) == 1);
        CHECK(result.distance.at(v2) == 2);
        CHECK(result.distance.at(v3) == 3);

        CHECK(result.discovery_order.size() == 4);
        CHECK(result.discovery_order[0] == v0);
    }

    SUBCASE("BFS with target") {
        auto result = algorithms::bfs(g, v0, v3);

        CHECK(result.target_found == true);
        CHECK(result.distance.at(v3) == 3);

        auto path = algorithms::reconstruct_path(result, v0, v3);
        REQUIRE(path.size() == 4);
        CHECK(path[0] == v0);
        CHECK(path[1] == v1);
        CHECK(path[2] == v2);
        CHECK(path[3] == v3);
    }
}

TEST_CASE("BFS - Tree structure") {
    Graph<void> g;

    //       0
    //      / \
    //     1   2
    //    / \   \
    //   3   4   5

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();
    auto v5 = g.add_vertex();

    g.add_edge(v0, v1);
    g.add_edge(v0, v2);
    g.add_edge(v1, v3);
    g.add_edge(v1, v4);
    g.add_edge(v2, v5);

    SUBCASE("BFS from root") {
        auto result = algorithms::bfs(g, v0);

        CHECK(result.distance.at(v0) == 0);
        CHECK(result.distance.at(v1) == 1);
        CHECK(result.distance.at(v2) == 1);
        CHECK(result.distance.at(v3) == 2);
        CHECK(result.distance.at(v4) == 2);
        CHECK(result.distance.at(v5) == 2);

        CHECK(result.discovery_order.size() == 6);
    }

    SUBCASE("Path from v3 to v5") {
        auto result = algorithms::bfs(g, v3, v5);

        CHECK(result.target_found == true);

        auto path = algorithms::reconstruct_path(result, v3, v5);
        REQUIRE(path.size() == 5);
        CHECK(path[0] == v3);
        CHECK(path[4] == v5);
        // Path should go: v3 -> v1 -> v0 -> v2 -> v5
    }
}

TEST_CASE("BFS - Disconnected graph") {
    Graph<void> g;

    // Component 1: 0 - 1
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    g.add_edge(v0, v1);

    // Component 2: 2 - 3
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    g.add_edge(v2, v3);

    SUBCASE("BFS from v0 doesn't reach v2") {
        auto result = algorithms::bfs(g, v0);

        CHECK(result.distance.find(v0) != result.distance.end());
        CHECK(result.distance.find(v1) != result.distance.end());
        CHECK(result.distance.find(v2) == result.distance.end());
        CHECK(result.distance.find(v3) == result.distance.end());

        CHECK(result.discovery_order.size() == 2);
    }

    SUBCASE("BFS with unreachable target") {
        auto result = algorithms::bfs(g, v0, v3);

        CHECK(result.target_found == false);

        auto path = algorithms::reconstruct_path(result, v0, v3);
        CHECK(path.empty());
    }
}

TEST_CASE("BFS - Cycle") {
    Graph<void> g;

    // Square: 0 - 1
    //         |   |
    //         3 - 2

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v0, v1);
    g.add_edge(v1, v2);
    g.add_edge(v2, v3);
    g.add_edge(v3, v0);

    SUBCASE("BFS finds shortest path") {
        auto result = algorithms::bfs(g, v0, v2);

        CHECK(result.target_found == true);
        CHECK(result.distance.at(v2) == 2);

        auto path = algorithms::reconstruct_path(result, v0, v2);
        REQUIRE(path.size() == 3);
        CHECK(path[0] == v0);
        CHECK(path[2] == v2);
        // Could be v0->v1->v2 or v0->v3->v2, both distance 2
    }
}

TEST_CASE("BFS - Single vertex") {
    Graph<void> g;
    auto v0 = g.add_vertex();

    auto result = algorithms::bfs(g, v0);

    CHECK(result.distance.at(v0) == 0);
    CHECK(result.discovery_order.size() == 1);
    CHECK(result.parent.empty());
}

TEST_CASE("BFS - Graph with properties") {
    struct Point {
        int x, y;
    };

    Graph<Point> g;

    auto v0 = g.add_vertex({0, 0});
    auto v1 = g.add_vertex({1, 0});
    auto v2 = g.add_vertex({2, 0});

    g.add_edge(v0, v1);
    g.add_edge(v1, v2);

    auto result = algorithms::bfs(g, v0);

    CHECK(result.distance.at(v0) == 0);
    CHECK(result.distance.at(v1) == 1);
    CHECK(result.distance.at(v2) == 2);

    // Verify we can still access vertex properties
    CHECK(g[v0].x == 0);
    CHECK(g[v1].x == 1);
    CHECK(g[v2].x == 2);
}
