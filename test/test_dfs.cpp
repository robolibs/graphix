#include "graphix/vertex/algorithms/dfs.hpp"
#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

using namespace graphix::vertex;

TEST_CASE("DFS - Simple path") {
    Graph<void> g;

    // Create simple path: 0 - 1 - 2 - 3
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v0, v1);
    g.add_edge(v1, v2);
    g.add_edge(v2, v3);

    SUBCASE("DFS from v0") {
        auto result = algorithms::dfs(g, v0);

        CHECK(result.discovery_time.at(v0) < result.discovery_time.at(v1));
        CHECK(result.discovery_time.at(v1) < result.discovery_time.at(v2));
        CHECK(result.discovery_time.at(v2) < result.discovery_time.at(v3));

        CHECK(result.preorder.size() == 4);
        CHECK(result.postorder.size() == 4);
        CHECK(result.preorder[0] == v0);
    }

    SUBCASE("DFS with target") {
        auto result = algorithms::dfs(g, v0, v3);

        CHECK(result.target_found == true);

        auto path = algorithms::reconstruct_path(result, v0, v3);
        REQUIRE(path.size() == 4);
        CHECK(path[0] == v0);
        CHECK(path[3] == v3);
    }

    SUBCASE("DFS iterative") {
        auto result = algorithms::dfs_iterative(g, v0);

        CHECK(result.preorder.size() == 4);
        CHECK(result.preorder[0] == v0);
    }
}

TEST_CASE("DFS - Tree structure") {
    Graph<void> g;

    //       0
    //      /  \
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

    SUBCASE("DFS from root") {
        auto result = algorithms::dfs(g, v0);

        CHECK(result.preorder.size() == 6);
        CHECK(result.postorder.size() == 6);

        // First vertex discovered should be root
        CHECK(result.preorder[0] == v0);

        // Last vertex finished should be root
        CHECK(result.postorder[5] == v0);
    }

    SUBCASE("Path from v3 to v5") {
        auto result = algorithms::dfs(g, v3, v5);

        CHECK(result.target_found == true);

        auto path = algorithms::reconstruct_path(result, v3, v5);
        CHECK(!path.empty());
        CHECK(path[0] == v3);
        CHECK(path[path.size() - 1] == v5);
    }
}

TEST_CASE("DFS - Disconnected graph") {
    Graph<void> g;

    // Component 1: 0 - 1
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    g.add_edge(v0, v1);

    // Component 2: 2 - 3
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    g.add_edge(v2, v3);

    SUBCASE("DFS from v0 doesn't reach v2") {
        auto result = algorithms::dfs(g, v0);

        CHECK(result.discovery_time.find(v0) != result.discovery_time.end());
        CHECK(result.discovery_time.find(v1) != result.discovery_time.end());
        CHECK(result.discovery_time.find(v2) == result.discovery_time.end());
        CHECK(result.discovery_time.find(v3) == result.discovery_time.end());

        CHECK(result.preorder.size() == 2);
    }

    SUBCASE("DFS with unreachable target") {
        auto result = algorithms::dfs(g, v0, v3);

        CHECK(result.target_found == false);

        auto path = algorithms::reconstruct_path(result, v0, v3);
        CHECK(path.empty());
    }
}

TEST_CASE("DFS - Cycle") {
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

    SUBCASE("DFS visits all vertices") {
        auto result = algorithms::dfs(g, v0);

        CHECK(result.preorder.size() == 4);
        CHECK(result.postorder.size() == 4);

        // All vertices should be discovered
        CHECK(result.discovery_time.size() == 4);
        CHECK(result.finish_time.size() == 4);
    }
}

TEST_CASE("DFS - Single vertex") {
    Graph<void> g;
    auto v0 = g.add_vertex();

    auto result = algorithms::dfs(g, v0);

    CHECK(result.preorder.size() == 1);
    CHECK(result.postorder.size() == 1);
    CHECK(result.parent.empty());
}

TEST_CASE("DFS - Graph with properties") {
    struct Point {
        int x, y;
    };

    Graph<Point> g;

    auto v0 = g.add_vertex({0, 0});
    auto v1 = g.add_vertex({1, 0});
    auto v2 = g.add_vertex({2, 0});

    g.add_edge(v0, v1);
    g.add_edge(v1, v2);

    auto result = algorithms::dfs(g, v0);

    CHECK(result.preorder.size() == 3);
    CHECK(result.postorder.size() == 3);

    // Verify we can still access vertex properties
    CHECK(g[v0].x == 0);
    CHECK(g[v1].x == 1);
    CHECK(g[v2].x == 2);
}
