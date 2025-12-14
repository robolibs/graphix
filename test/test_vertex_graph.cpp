#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
    bool operator==(const Point &other) const { return x == other.x && y == other.y; }
};

// ============================================================================
// Step 1: Basic Graph Structure with Vertex Properties
// ============================================================================

TEST_CASE("Graph without properties - add vertices") {
    graphix::vertex::Graph<void> g;

    CHECK(g.vertex_count() == 0);

    auto v1 = g.add_vertex();
    CHECK(g.vertex_count() == 1);
    CHECK(g.has_vertex(v1));

    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    CHECK(g.vertex_count() == 3);
    CHECK(g.has_vertex(v2));
    CHECK(g.has_vertex(v3));
}

TEST_CASE("Graph with int properties") {
    graphix::vertex::Graph<int> g;

    CHECK(g.vertex_count() == 0);

    auto v1 = g.add_vertex(10);
    CHECK(g.vertex_count() == 1);
    CHECK(g[v1] == 10);

    auto v2 = g.add_vertex(20);
    CHECK(g[v2] == 20);
    CHECK(g.vertex_count() == 2);
}

TEST_CASE("Graph with Point properties") {
    graphix::vertex::Graph<Point> g;

    auto v1 = g.add_vertex(Point(1.0, 2.0));
    auto v2 = g.add_vertex(Point(3.0, 4.0));
    auto v3 = g.add_vertex(Point(5.0, 6.0));

    CHECK(g.vertex_count() == 3);
    CHECK(g[v1] == Point(1.0, 2.0));
    CHECK(g[v2] == Point(3.0, 4.0));
    CHECK(g[v3] == Point(5.0, 6.0));
}

TEST_CASE("Modify vertex properties") {
    graphix::vertex::Graph<int> g;

    auto v1 = g.add_vertex(10);
    CHECK(g[v1] == 10);

    g[v1] = 99;
    CHECK(g[v1] == 99);
}

TEST_CASE("Const access to vertex properties") {
    graphix::vertex::Graph<int> g;
    auto v1 = g.add_vertex(42);

    const auto &const_g = g;
    CHECK(const_g[v1] == 42);
}

TEST_CASE("Has vertex check") {
    graphix::vertex::Graph<int> g;

    auto v1 = g.add_vertex(10);
    auto v2 = g.add_vertex(20);

    CHECK(g.has_vertex(v1));
    CHECK(g.has_vertex(v2));
    CHECK_FALSE(g.has_vertex(999));
}

// ============================================================================
// Step 2: Edge Support with Weights
// ============================================================================

TEST_CASE("Add edges without properties") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    CHECK(g.edge_count() == 0);

    auto e1 = g.add_edge(v1, v2);
    CHECK(g.edge_count() == 1);

    auto e2 = g.add_edge(v2, v3, 2.5);
    CHECK(g.edge_count() == 2);

    auto e3 = g.add_edge(v1, v3, 3.0);
    CHECK(g.edge_count() == 3);
}

TEST_CASE("Check edge existence") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v1, v2);
    g.add_edge(v2, v3);

    CHECK(g.has_edge(v1, v2));
    CHECK(g.has_edge(v2, v1)); // Undirected
    CHECK(g.has_edge(v2, v3));
    CHECK(g.has_edge(v3, v2)); // Undirected
    CHECK_FALSE(g.has_edge(v1, v3));
    CHECK_FALSE(g.has_edge(v3, v1));
}

TEST_CASE("Get and set edge weights") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    auto e1 = g.add_edge(v1, v2, 1.5);
    auto e2 = g.add_edge(v2, v3, 2.5);

    CHECK(g.get_weight(e1) == 1.5);
    CHECK(g.get_weight(e2) == 2.5);

    g.set_weight(e1, 10.0);
    CHECK(g.get_weight(e1) == 10.0);

    g.set_weight(e2, 20.0);
    CHECK(g.get_weight(e2) == 20.0);
}

TEST_CASE("Default edge weight") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    auto e = g.add_edge(v1, v2); // No weight specified, should default to 1.0
    CHECK(g.get_weight(e) == 1.0);
}

TEST_CASE("Add edges with vertex properties") {
    graphix::vertex::Graph<int> g;

    auto v1 = g.add_vertex(10);
    auto v2 = g.add_vertex(20);
    auto v3 = g.add_vertex(30);

    CHECK(g.edge_count() == 0);

    g.add_edge(v1, v2, 1.5);
    CHECK(g.edge_count() == 1);

    g.add_edge(v2, v3, 2.5);
    CHECK(g.edge_count() == 2);

    CHECK(g.has_edge(v1, v2));
    CHECK(g.has_edge(v2, v3));
    CHECK_FALSE(g.has_edge(v1, v3));
}

TEST_CASE("Undirected edge behavior") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    auto e = g.add_edge(v1, v2, 5.0);

    // Both directions should exist with same weight
    CHECK(g.has_edge(v1, v2));
    CHECK(g.has_edge(v2, v1));
    CHECK(g.get_weight(e) == 5.0);

    // Updating weight should affect both directions
    g.set_weight(e, 7.0);
    CHECK(g.get_weight(e) == 7.0);
}

TEST_CASE("Edge count tracking") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    CHECK(g.edge_count() == 0);

    g.add_edge(v1, v2);
    CHECK(g.edge_count() == 1);

    g.add_edge(v2, v3);
    CHECK(g.edge_count() == 2);

    g.add_edge(v3, v4);
    CHECK(g.edge_count() == 3);

    g.add_edge(v1, v4);
    CHECK(g.edge_count() == 4);
}
