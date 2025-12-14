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
