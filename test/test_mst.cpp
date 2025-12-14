#include "graphix/vertex/algorithms/mst.hpp"
#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

using namespace graphix::vertex;

TEST_CASE("MST - Simple triangle") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    g.add_edge(v0, v1, 1.0);
    g.add_edge(v1, v2, 2.0);
    g.add_edge(v2, v0, 3.0);

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 2);   // MST has n-1 edges
    CHECK(result.total_weight == 3.0); // Should pick edges with weights 1 and 2
}

TEST_CASE("MST - Simple path") {
    Graph<void> g;

    // 0 --1-- 1 --2-- 2 --3-- 3
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v0, v1, 1.0);
    g.add_edge(v1, v2, 2.0);
    g.add_edge(v2, v3, 3.0);

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 3);
    CHECK(result.total_weight == 6.0);
}

TEST_CASE("MST - Square with diagonal") {
    Graph<void> g;

    //  0 --1-- 1
    //  |   X   |
    //  4   5   2
    //  |       |
    //  3 --3-- 2

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v0, v1, 1.0);
    g.add_edge(v1, v2, 2.0);
    g.add_edge(v2, v3, 3.0);
    g.add_edge(v3, v0, 4.0);
    g.add_edge(v0, v2, 5.0); // Diagonal

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 3);
    CHECK(result.total_weight == 6.0); // Should be 1 + 2 + 3 (avoiding diagonal and edge with weight 4)
}

TEST_CASE("MST - Complete graph K4") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    // All possible edges with different weights
    g.add_edge(v0, v1, 1.0);
    g.add_edge(v0, v2, 2.0);
    g.add_edge(v0, v3, 3.0);
    g.add_edge(v1, v2, 4.0);
    g.add_edge(v1, v3, 5.0);
    g.add_edge(v2, v3, 6.0);

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 3);
    CHECK(result.total_weight == 6.0); // Should pick edges: 1, 2, 3
}

TEST_CASE("MST - Disconnected graph") {
    Graph<void> g;

    // Component 1: 0 - 1
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    g.add_edge(v0, v1, 1.0);

    // Component 2: 2 - 3
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    g.add_edge(v2, v3, 2.0);

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == false); // Can't span disconnected graph
    CHECK(result.edges.size() == 1);         // Only edges from one component
}

TEST_CASE("MST - Single vertex") {
    Graph<void> g;
    auto v0 = g.add_vertex();

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.empty());
    CHECK(result.total_weight == 0.0);
}

TEST_CASE("MST - Two vertices") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    g.add_edge(v0, v1, 5.0);

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 1);
    CHECK(result.total_weight == 5.0);
}

TEST_CASE("MST - With specific start vertex") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    g.add_edge(v0, v1, 1.0);
    g.add_edge(v1, v2, 2.0);
    g.add_edge(v2, v0, 3.0);

    // Start from v1
    auto result = algorithms::prim_mst(g, v1);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 2);
    CHECK(result.total_weight == 3.0);
}

TEST_CASE("MST - Equal weights") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    // All edges have same weight
    g.add_edge(v0, v1, 1.0);
    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 1.0);
    g.add_edge(v3, v0, 1.0);

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 3);
    CHECK(result.total_weight == 3.0);
}

TEST_CASE("MST - Graph with properties") {
    struct Point {
        double x, y;
    };

    Graph<Point> g;

    auto v0 = g.add_vertex({0, 0});
    auto v1 = g.add_vertex({1, 0});
    auto v2 = g.add_vertex({0, 1});

    g.add_edge(v0, v1, 1.0);
    g.add_edge(v1, v2, 2.0);
    g.add_edge(v2, v0, 3.0);

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 2);
    CHECK(result.total_weight == 3.0);

    // Verify properties still accessible
    CHECK(g[v0].x == 0);
    CHECK(g[v1].x == 1);
}

TEST_CASE("MST - Complex graph") {
    Graph<void> g;

    //      1
    //   0----1
    //   |\   |\2
    //  4| \5 | \
    //   |  \ |  2
    //   3---\-3
    //     3  \|6
    //         4

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    g.add_edge(v0, v1, 1.0);
    g.add_edge(v1, v2, 2.0);
    g.add_edge(v0, v3, 4.0);
    g.add_edge(v3, v4, 3.0);
    g.add_edge(v0, v4, 5.0);
    g.add_edge(v1, v4, 6.0);

    auto result = algorithms::prim_mst(g);

    CHECK(result.is_spanning_tree == true);
    CHECK(result.edges.size() == 4);
    // Minimum should be: 1 (0-1) + 2 (1-2) + 3 (3-4) + 4 (0-3) = 10
    // Or: 1 (0-1) + 2 (1-2) + 5 (0-4) + 3 (4-3) = 11
    // Or: 1 (0-1) + 2 (1-2) + 4 (0-3) + 3 (3-4) = 10
    CHECK(result.total_weight == 10.0);
}
