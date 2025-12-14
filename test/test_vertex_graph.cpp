#include "graphix/vertex/algorithms/dijkstra.hpp"
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

// ============================================================================
// Step 3: Adjacency and Neighbor Queries
// ============================================================================

TEST_CASE("Get neighbors of a vertex") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    g.add_edge(v1, v2);
    g.add_edge(v1, v3);
    g.add_edge(v1, v4);

    auto neighbors = g.neighbors(v1);
    CHECK(neighbors.size() == 3);

    // Check all neighbors are present (order doesn't matter)
    CHECK(std::find(neighbors.begin(), neighbors.end(), v2) != neighbors.end());
    CHECK(std::find(neighbors.begin(), neighbors.end(), v3) != neighbors.end());
    CHECK(std::find(neighbors.begin(), neighbors.end(), v4) != neighbors.end());
}

TEST_CASE("Neighbors on empty vertex") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    // v1 has no edges
    auto neighbors = g.neighbors(v1);
    CHECK(neighbors.empty());
}

TEST_CASE("Neighbors with undirected edges") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v1, v2);
    g.add_edge(v2, v3);

    // v2 should be neighbor of v1
    auto n1 = g.neighbors(v1);
    CHECK(n1.size() == 1);
    CHECK(n1[0] == v2);

    // v1 and v3 should be neighbors of v2
    auto n2 = g.neighbors(v2);
    CHECK(n2.size() == 2);
    CHECK(std::find(n2.begin(), n2.end(), v1) != n2.end());
    CHECK(std::find(n2.begin(), n2.end(), v3) != n2.end());

    // v2 should be neighbor of v3
    auto n3 = g.neighbors(v3);
    CHECK(n3.size() == 1);
    CHECK(n3[0] == v2);
}

TEST_CASE("Vertex degree") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    // Initially all vertices have degree 0
    CHECK(g.degree(v1) == 0);
    CHECK(g.degree(v2) == 0);

    // Add edges
    g.add_edge(v1, v2);
    CHECK(g.degree(v1) == 1);
    CHECK(g.degree(v2) == 1);

    g.add_edge(v1, v3);
    CHECK(g.degree(v1) == 2);
    CHECK(g.degree(v3) == 1);

    g.add_edge(v1, v4);
    CHECK(g.degree(v1) == 3); // v1 connected to v2, v3, v4
    CHECK(g.degree(v4) == 1);

    g.add_edge(v2, v3);
    CHECK(g.degree(v2) == 2); // v2 connected to v1, v3
    CHECK(g.degree(v3) == 2); // v3 connected to v1, v2
}

TEST_CASE("Neighbors with vertex properties") {
    graphix::vertex::Graph<int> g;

    auto v1 = g.add_vertex(100);
    auto v2 = g.add_vertex(200);
    auto v3 = g.add_vertex(300);

    g.add_edge(v1, v2);
    g.add_edge(v1, v3);

    auto neighbors = g.neighbors(v1);
    CHECK(neighbors.size() == 2);

    // Verify we can still access properties
    CHECK(g[v1] == 100);
    CHECK(g[v2] == 200);
}

TEST_CASE("Degree with vertex properties") {
    graphix::vertex::Graph<Point> g;

    auto v1 = g.add_vertex(Point(1.0, 2.0));
    auto v2 = g.add_vertex(Point(3.0, 4.0));
    auto v3 = g.add_vertex(Point(5.0, 6.0));

    CHECK(g.degree(v1) == 0);

    g.add_edge(v1, v2, 1.5);
    CHECK(g.degree(v1) == 1);
    CHECK(g.degree(v2) == 1);

    g.add_edge(v1, v3, 2.5);
    CHECK(g.degree(v1) == 2);
}

TEST_CASE("Star graph topology") {
    graphix::vertex::Graph<void> g;

    auto center = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    // Create star: center connected to all others
    g.add_edge(center, v1);
    g.add_edge(center, v2);
    g.add_edge(center, v3);
    g.add_edge(center, v4);

    // Center has degree 4
    CHECK(g.degree(center) == 4);

    // All outer vertices have degree 1
    CHECK(g.degree(v1) == 1);
    CHECK(g.degree(v2) == 1);
    CHECK(g.degree(v3) == 1);
    CHECK(g.degree(v4) == 1);

    // Center's neighbors are all outer vertices
    auto neighbors = g.neighbors(center);
    CHECK(neighbors.size() == 4);
}

// ============================================================================
// Step 4: Iterators
// ============================================================================

TEST_CASE("Iterate over vertices without properties") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    auto vertices = g.vertices();
    CHECK(vertices.size() == 3);

    // Check all vertices are present
    CHECK(std::find(vertices.begin(), vertices.end(), v1) != vertices.end());
    CHECK(std::find(vertices.begin(), vertices.end(), v2) != vertices.end());
    CHECK(std::find(vertices.begin(), vertices.end(), v3) != vertices.end());
}

TEST_CASE("Iterate over vertices with properties") {
    graphix::vertex::Graph<int> g;

    auto v1 = g.add_vertex(100);
    auto v2 = g.add_vertex(200);
    auto v3 = g.add_vertex(300);

    auto vertices = g.vertices();
    CHECK(vertices.size() == 3);

    // Can access properties while iterating
    int sum = 0;
    for (auto v : vertices) {
        sum += g[v];
    }
    CHECK(sum == 600);
}

TEST_CASE("Iterate over empty graph") {
    graphix::vertex::Graph<void> g;

    auto vertices = g.vertices();
    CHECK(vertices.empty());

    auto edges = g.edges();
    CHECK(edges.empty());
}

TEST_CASE("Iterate over edges") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    auto e1 = g.add_edge(v1, v2, 1.5);
    auto e2 = g.add_edge(v2, v3, 2.5);
    auto e3 = g.add_edge(v1, v3, 3.5);

    auto edges = g.edges();
    CHECK(edges.size() == 3);

    // Verify edge data
    bool found_e1 = false, found_e2 = false, found_e3 = false;

    for (const auto &edge : edges) {
        if (edge.id == e1) {
            CHECK(edge.weight == 1.5);
            CHECK(((edge.source == v1 && edge.target == v2) || (edge.source == v2 && edge.target == v1)));
            found_e1 = true;
        } else if (edge.id == e2) {
            CHECK(edge.weight == 2.5);
            CHECK(((edge.source == v2 && edge.target == v3) || (edge.source == v3 && edge.target == v2)));
            found_e2 = true;
        } else if (edge.id == e3) {
            CHECK(edge.weight == 3.5);
            CHECK(((edge.source == v1 && edge.target == v3) || (edge.source == v3 && edge.target == v1)));
            found_e3 = true;
        }
    }

    CHECK(found_e1);
    CHECK(found_e2);
    CHECK(found_e3);
}

TEST_CASE("Edge iteration avoids duplicates") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    g.add_edge(v1, v2);
    g.add_edge(v2, v3);
    g.add_edge(v3, v4);
    g.add_edge(v4, v1);

    auto edges = g.edges();

    // Should have exactly 4 edges, not 8 (undirected graph stores each edge twice internally)
    CHECK(edges.size() == 4);
    CHECK(g.edge_count() == 4);
}

TEST_CASE("Iterate and compute total weight") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 2.0);
    g.add_edge(v1, v3, 3.0);

    auto edges = g.edges();
    double total_weight = 0.0;
    for (const auto &edge : edges) {
        total_weight += edge.weight;
    }

    CHECK(total_weight == 6.0);
}

TEST_CASE("Range-based for loop over vertices") {
    graphix::vertex::Graph<int> g;

    g.add_vertex(10);
    g.add_vertex(20);
    g.add_vertex(30);

    int count = 0;
    for (auto v : g.vertices()) {
        CHECK(g.has_vertex(v));
        count++;
    }
    CHECK(count == 3);
}

TEST_CASE("Range-based for loop over edges") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 2.0);

    int count = 0;
    for (const auto &edge : g.edges()) {
        CHECK(edge.weight > 0.0);
        count++;
    }
    CHECK(count == 2);
}

TEST_CASE("Vertices iteration with Point properties") {
    graphix::vertex::Graph<Point> g;

    auto v1 = g.add_vertex(Point(1.0, 2.0));
    auto v2 = g.add_vertex(Point(3.0, 4.0));
    auto v3 = g.add_vertex(Point(5.0, 6.0));

    auto vertices = g.vertices();
    CHECK(vertices.size() == 3);

    // Verify properties are accessible
    for (auto v : vertices) {
        CHECK(g[v].x > 0.0);
        CHECK(g[v].y > 0.0);
    }
}

// ============================================================================
// Step 5: Dijkstra's Shortest Path
// ============================================================================

TEST_CASE("Simple shortest path") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 2.0);

    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v3);

    CHECK(result.found);
    CHECK(result.distance == 3.0);
    REQUIRE(result.path.size() == 3);
    CHECK(result.path[0] == v1);
    CHECK(result.path[1] == v2);
    CHECK(result.path[2] == v3);
}

TEST_CASE("Shortest path same vertex") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();

    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v1);

    CHECK(result.found);
    CHECK(result.distance == 0.0);
    REQUIRE(result.path.size() == 1);
    CHECK(result.path[0] == v1);
}

TEST_CASE("Shortest path no path exists") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    // v1-v2 connected, v3-v4 connected, but no path between groups
    g.add_edge(v1, v2, 1.0);
    g.add_edge(v3, v4, 1.0);

    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v3);

    CHECK_FALSE(result.found);
    CHECK(result.distance == std::numeric_limits<double>::infinity());
}

TEST_CASE("Shortest path chooses minimum weight") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    // Create two paths from v1 to v4:
    // v1 -> v2 -> v4 (cost 1 + 1 = 2)
    // v1 -> v3 -> v4 (cost 5 + 5 = 10)
    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v4, 1.0);
    g.add_edge(v1, v3, 5.0);
    g.add_edge(v3, v4, 5.0);

    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v4);

    CHECK(result.found);
    CHECK(result.distance == 2.0);
    REQUIRE(result.path.size() == 3);
    CHECK(result.path[0] == v1);
    CHECK(result.path[1] == v2);
    CHECK(result.path[2] == v4);
}

TEST_CASE("Shortest path with vertex properties") {
    graphix::vertex::Graph<int> g;

    auto v1 = g.add_vertex(100);
    auto v2 = g.add_vertex(200);
    auto v3 = g.add_vertex(300);

    g.add_edge(v1, v2, 1.5);
    g.add_edge(v2, v3, 2.5);

    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v3);

    CHECK(result.found);
    CHECK(result.distance == 4.0);
    REQUIRE(result.path.size() == 3);

    // Verify we can access properties along path
    CHECK(g[result.path[0]] == 100);
    CHECK(g[result.path[1]] == 200);
    CHECK(g[result.path[2]] == 300);
}

TEST_CASE("Shortest path in complete graph") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    // Complete graph with all vertices connected
    g.add_edge(v1, v2, 1.0);
    g.add_edge(v1, v3, 3.0);
    g.add_edge(v1, v4, 5.0);
    g.add_edge(v2, v3, 1.0);
    g.add_edge(v2, v4, 7.0);
    g.add_edge(v3, v4, 1.0);

    // Shortest path v1 to v4 should be v1 -> v2 -> v3 -> v4 (1 + 1 + 1 = 3)
    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v4);

    CHECK(result.found);
    CHECK(result.distance == 3.0);
}

TEST_CASE("Shortest path with Point properties") {
    graphix::vertex::Graph<Point> g;

    auto v1 = g.add_vertex(Point(0.0, 0.0));
    auto v2 = g.add_vertex(Point(1.0, 0.0));
    auto v3 = g.add_vertex(Point(2.0, 0.0));

    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 1.0);

    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v3);

    CHECK(result.found);
    CHECK(result.distance == 2.0);
    CHECK(result.path.size() == 3);
}

TEST_CASE("Shortest path linear chain") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();
    auto v5 = g.add_vertex();

    // Linear chain: v1 - v2 - v3 - v4 - v5
    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 1.0);
    g.add_edge(v3, v4, 1.0);
    g.add_edge(v4, v5, 1.0);

    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v5);

    CHECK(result.found);
    CHECK(result.distance == 4.0);
    REQUIRE(result.path.size() == 5);
    CHECK(result.path[0] == v1);
    CHECK(result.path[4] == v5);
}

TEST_CASE("Shortest path with varying weights") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    // Diamond shape with different weights
    g.add_edge(v1, v2, 1.0);
    g.add_edge(v1, v3, 4.0);
    g.add_edge(v2, v4, 2.0);
    g.add_edge(v3, v4, 1.0);

    // Shortest v1 to v4: v1 -> v2 -> v4 (1 + 2 = 3)
    // vs v1 -> v3 -> v4 (4 + 1 = 5)
    auto result = graphix::vertex::algorithms::dijkstra(g, v1, v4);

    CHECK(result.found);
    CHECK(result.distance == 3.0);
}

// ============================================================================
// Step 6: Graph Modification
// ============================================================================

TEST_CASE("Clear graph") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 2.0);

    CHECK(g.vertex_count() == 3);
    CHECK(g.edge_count() == 2);

    g.clear();

    CHECK(g.vertex_count() == 0);
    CHECK(g.edge_count() == 0);
}

TEST_CASE("Remove edge by ID") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    auto e1 = g.add_edge(v1, v2, 1.0);
    auto e2 = g.add_edge(v2, v3, 2.0);

    CHECK(g.edge_count() == 2);
    CHECK(g.has_edge(v1, v2));

    g.remove_edge(e1);

    CHECK(g.edge_count() == 1);
    CHECK_FALSE(g.has_edge(v1, v2));
    CHECK(g.has_edge(v2, v3));
}

TEST_CASE("Remove edge by vertices") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 2.0);

    CHECK(g.has_edge(v1, v2));

    g.remove_edge(v1, v2);

    CHECK_FALSE(g.has_edge(v1, v2));
    CHECK(g.has_edge(v2, v3));
    CHECK(g.edge_count() == 1);
}

TEST_CASE("Remove vertex") {
    graphix::vertex::Graph<int> g;

    auto v1 = g.add_vertex(10);
    auto v2 = g.add_vertex(20);
    auto v3 = g.add_vertex(30);

    g.add_edge(v1, v2, 1.0);
    g.add_edge(v2, v3, 2.0);
    g.add_edge(v1, v3, 3.0);

    CHECK(g.vertex_count() == 3);
    CHECK(g.edge_count() == 3);

    g.remove_vertex(v2);

    CHECK(g.vertex_count() == 2);
    CHECK_FALSE(g.has_vertex(v2));
    CHECK(g.has_vertex(v1));
    CHECK(g.has_vertex(v3));

    // Edges involving v2 should be removed
    CHECK_FALSE(g.has_edge(v1, v2));
    CHECK_FALSE(g.has_edge(v2, v3));
    // Edge between v1 and v3 should remain
    CHECK(g.has_edge(v1, v3));
    CHECK(g.edge_count() == 1);
}

TEST_CASE("Remove vertex with no edges") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    CHECK(g.vertex_count() == 2);

    g.remove_vertex(v1);

    CHECK(g.vertex_count() == 1);
    CHECK_FALSE(g.has_vertex(v1));
    CHECK(g.has_vertex(v2));
}

TEST_CASE("Clear graph with properties") {
    graphix::vertex::Graph<Point> g;

    auto v1 = g.add_vertex(Point(1.0, 2.0));
    auto v2 = g.add_vertex(Point(3.0, 4.0));

    g.add_edge(v1, v2, 5.0);

    g.clear();

    CHECK(g.vertex_count() == 0);
    CHECK(g.edge_count() == 0);
}

// ============================================================================
// Step 7: Boost-style Free Functions
// ============================================================================

TEST_CASE("Boost-style num_vertices") {
    graphix::vertex::Graph<void> g;

    CHECK(graphix::vertex::num_vertices(g) == 0);

    g.add_vertex();
    g.add_vertex();
    g.add_vertex();

    CHECK(graphix::vertex::num_vertices(g) == 3);
}

TEST_CASE("Boost-style num_vertices with properties") {
    graphix::vertex::Graph<int> g;

    CHECK(graphix::vertex::num_vertices(g) == 0);

    g.add_vertex(10);
    g.add_vertex(20);

    CHECK(graphix::vertex::num_vertices(g) == 2);
}

TEST_CASE("Boost-style num_edges") {
    graphix::vertex::Graph<void> g;

    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    CHECK(graphix::vertex::num_edges(g) == 0);

    g.add_edge(v1, v2);
    CHECK(graphix::vertex::num_edges(g) == 1);

    g.add_edge(v2, v3);
    CHECK(graphix::vertex::num_edges(g) == 2);
}

TEST_CASE("Boost-style add_vertex without properties") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);

    CHECK(graphix::vertex::num_vertices(g) == 2);
    CHECK(g.has_vertex(v1));
    CHECK(g.has_vertex(v2));
}

TEST_CASE("Boost-style add_vertex with properties") {
    graphix::vertex::Graph<int> g;

    auto v1 = graphix::vertex::add_vertex(100, g);
    auto v2 = graphix::vertex::add_vertex(200, g);

    CHECK(graphix::vertex::num_vertices(g) == 2);
    CHECK(g[v1] == 100);
    CHECK(g[v2] == 200);
}

TEST_CASE("Boost-style add_edge with weight") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);

    auto e = graphix::vertex::add_edge(v1, v2, 5.0, g);

    CHECK(graphix::vertex::num_edges(g) == 1);
    CHECK(g.has_edge(v1, v2));
    CHECK(g.get_weight(e) == 5.0);
}

TEST_CASE("Boost-style add_edge without weight") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);

    auto e = graphix::vertex::add_edge(v1, v2, g);

    CHECK(graphix::vertex::num_edges(g) == 1);
    CHECK(g.get_weight(e) == 1.0); // Default weight
}

TEST_CASE("Boost-style degree") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);
    auto v3 = graphix::vertex::add_vertex(g);

    CHECK(graphix::vertex::degree(v1, g) == 0);

    graphix::vertex::add_edge(v1, v2, g);
    CHECK(graphix::vertex::degree(v1, g) == 1);

    graphix::vertex::add_edge(v1, v3, g);
    CHECK(graphix::vertex::degree(v1, g) == 2);
}

TEST_CASE("Boost-style neighbors") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);
    auto v3 = graphix::vertex::add_vertex(g);

    graphix::vertex::add_edge(v1, v2, g);
    graphix::vertex::add_edge(v1, v3, g);

    auto n = graphix::vertex::neighbors(v1, g);
    CHECK(n.size() == 2);
    CHECK(std::find(n.begin(), n.end(), v2) != n.end());
    CHECK(std::find(n.begin(), n.end(), v3) != n.end());
}

TEST_CASE("Boost-style vertices") {
    graphix::vertex::Graph<int> g;

    auto v1 = graphix::vertex::add_vertex(100, g);
    auto v2 = graphix::vertex::add_vertex(200, g);
    auto v3 = graphix::vertex::add_vertex(300, g);

    auto verts = graphix::vertex::vertices(g);
    CHECK(verts.size() == 3);
    CHECK(std::find(verts.begin(), verts.end(), v1) != verts.end());
    CHECK(std::find(verts.begin(), verts.end(), v2) != verts.end());
    CHECK(std::find(verts.begin(), verts.end(), v3) != verts.end());
}

TEST_CASE("Boost-style edges") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);
    auto v3 = graphix::vertex::add_vertex(g);

    auto e1 = graphix::vertex::add_edge(v1, v2, 1.5, g);
    auto e2 = graphix::vertex::add_edge(v2, v3, 2.5, g);

    auto edge_list = graphix::vertex::edges(g);
    CHECK(edge_list.size() == 2);

    bool found_e1 = false, found_e2 = false;
    for (const auto &edge : edge_list) {
        if (edge.id == e1) {
            CHECK(edge.weight == 1.5);
            found_e1 = true;
        } else if (edge.id == e2) {
            CHECK(edge.weight == 2.5);
            found_e2 = true;
        }
    }
    CHECK(found_e1);
    CHECK(found_e2);
}

TEST_CASE("Boost-style clear_graph") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);
    graphix::vertex::add_edge(v1, v2, g);

    CHECK(graphix::vertex::num_vertices(g) == 2);
    CHECK(graphix::vertex::num_edges(g) == 1);

    graphix::vertex::clear_graph(g);

    CHECK(graphix::vertex::num_vertices(g) == 0);
    CHECK(graphix::vertex::num_edges(g) == 0);
}

TEST_CASE("Boost-style remove_vertex") {
    graphix::vertex::Graph<int> g;

    auto v1 = graphix::vertex::add_vertex(10, g);
    auto v2 = graphix::vertex::add_vertex(20, g);
    auto v3 = graphix::vertex::add_vertex(30, g);

    graphix::vertex::add_edge(v1, v2, g);
    graphix::vertex::add_edge(v2, v3, g);

    CHECK(graphix::vertex::num_vertices(g) == 3);

    graphix::vertex::remove_vertex(v2, g);

    CHECK(graphix::vertex::num_vertices(g) == 2);
    CHECK_FALSE(g.has_vertex(v2));
    CHECK(g.has_vertex(v1));
    CHECK(g.has_vertex(v3));
}

TEST_CASE("Boost-style remove_edge by ID") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);
    auto v3 = graphix::vertex::add_vertex(g);

    auto e1 = graphix::vertex::add_edge(v1, v2, g);
    auto e2 = graphix::vertex::add_edge(v2, v3, g);

    CHECK(graphix::vertex::num_edges(g) == 2);

    graphix::vertex::remove_edge(e1, g);

    CHECK(graphix::vertex::num_edges(g) == 1);
    CHECK_FALSE(g.has_edge(v1, v2));
    CHECK(g.has_edge(v2, v3));
}

TEST_CASE("Boost-style remove_edge by vertices") {
    graphix::vertex::Graph<void> g;

    auto v1 = graphix::vertex::add_vertex(g);
    auto v2 = graphix::vertex::add_vertex(g);
    auto v3 = graphix::vertex::add_vertex(g);

    graphix::vertex::add_edge(v1, v2, g);
    graphix::vertex::add_edge(v2, v3, g);

    CHECK(graphix::vertex::num_edges(g) == 2);

    graphix::vertex::remove_edge(v1, v2, g);

    CHECK(graphix::vertex::num_edges(g) == 1);
    CHECK_FALSE(g.has_edge(v1, v2));
    CHECK(g.has_edge(v2, v3));
}

TEST_CASE("Boost-style mixed usage") {
    // Test mixing member functions and free functions
    graphix::vertex::Graph<int> g;

    auto v1 = graphix::vertex::add_vertex(100, g); // Free function
    auto v2 = g.add_vertex(200);                   // Member function
    auto v3 = graphix::vertex::add_vertex(300, g); // Free function

    auto e1 = g.add_edge(v1, v2);                   // Member function
    auto e2 = graphix::vertex::add_edge(v2, v3, g); // Free function

    CHECK(graphix::vertex::num_vertices(g) == 3); // Free function
    CHECK(g.edge_count() == 2);                   // Member function

    CHECK(graphix::vertex::degree(v2, g) == 2); // Free function
    CHECK(g.degree(v1) == 1);                   // Member function
}
