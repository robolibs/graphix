#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

using namespace graphix::vertex;

TEST_CASE("Graph Copy - Copy constructor") {
    Graph<void> g1;

    auto v0 = g1.add_vertex();
    auto v1 = g1.add_vertex();
    auto v2 = g1.add_vertex();

    g1.add_edge(v0, v1, 1.0);
    g1.add_edge(v1, v2, 2.0);

    // Copy construct
    Graph<void> g2(g1);

    SUBCASE("Original and copy have same structure") {
        CHECK(g2.vertex_count() == 3);
        CHECK(g2.edge_count() == 2);
        CHECK(g2.has_edge(v0, v1) == true);
        CHECK(g2.has_edge(v1, v2) == true);
    }

    SUBCASE("Copy is independent from original") {
        // Modify original
        auto v3 = g1.add_vertex();
        g1.add_edge(v2, v3, 3.0);

        // Copy should be unchanged
        CHECK(g1.vertex_count() == 4);
        CHECK(g1.edge_count() == 3);

        CHECK(g2.vertex_count() == 3);
        CHECK(g2.edge_count() == 2);
    }

    SUBCASE("Modify copy doesn't affect original") {
        // Modify copy
        auto v3 = g2.add_vertex();
        g2.add_edge(v2, v3, 3.0);

        // Original should be unchanged
        CHECK(g2.vertex_count() == 4);
        CHECK(g2.edge_count() == 3);

        CHECK(g1.vertex_count() == 3);
        CHECK(g1.edge_count() == 2);
    }
}

TEST_CASE("Graph Copy - Copy assignment") {
    Graph<void> g1;

    auto v0 = g1.add_vertex();
    auto v1 = g1.add_vertex();
    g1.add_edge(v0, v1, 5.0);

    Graph<void> g2;
    auto v2 = g2.add_vertex(); // Pre-existing data

    // Copy assign
    g2 = g1;

    CHECK(g2.vertex_count() == 2);
    CHECK(g2.edge_count() == 1);
    CHECK(g2.has_edge(v0, v1) == true);
}

TEST_CASE("Graph Move - Move constructor") {
    Graph<void> g1;

    auto v0 = g1.add_vertex();
    auto v1 = g1.add_vertex();
    auto v2 = g1.add_vertex();
    g1.add_edge(v0, v1, 1.0);
    g1.add_edge(v1, v2, 2.0);

    // Move construct
    Graph<void> g2(std::move(g1));

    SUBCASE("Moved-to graph has the data") {
        CHECK(g2.vertex_count() == 3);
        CHECK(g2.edge_count() == 2);
        CHECK(g2.has_edge(v0, v1) == true);
        CHECK(g2.has_edge(v1, v2) == true);
    }

    // Note: g1 is now in moved-from state, don't use it
}

TEST_CASE("Graph Move - Move assignment") {
    Graph<void> g1;

    auto v0 = g1.add_vertex();
    auto v1 = g1.add_vertex();
    g1.add_edge(v0, v1, 5.0);

    Graph<void> g2;
    auto v2 = g2.add_vertex(); // Pre-existing data

    // Move assign
    g2 = std::move(g1);

    CHECK(g2.vertex_count() == 2);
    CHECK(g2.edge_count() == 1);
    CHECK(g2.has_edge(v0, v1) == true);

    // Note: g1 is now in moved-from state
}

TEST_CASE("Graph Copy - With properties") {
    struct Point {
        double x, y;
    };

    Graph<Point> g1;

    auto v0 = g1.add_vertex({0.0, 0.0});
    auto v1 = g1.add_vertex({1.0, 1.0});
    auto v2 = g1.add_vertex({2.0, 2.0});

    g1.add_edge(v0, v1, 1.0);
    g1.add_edge(v1, v2, 2.0);

    // Copy construct
    Graph<Point> g2(g1);

    SUBCASE("Properties are copied") {
        CHECK(g2[v0].x == 0.0);
        CHECK(g2[v0].y == 0.0);
        CHECK(g2[v1].x == 1.0);
        CHECK(g2[v2].x == 2.0);
    }

    SUBCASE("Modifying copy properties doesn't affect original") {
        g2[v0].x = 99.0;

        CHECK(g2[v0].x == 99.0);
        CHECK(g1[v0].x == 0.0); // Original unchanged
    }
}

TEST_CASE("Graph Copy - Directed edges preserved") {
    Graph<void> g1;

    auto v0 = g1.add_vertex();
    auto v1 = g1.add_vertex();
    auto v2 = g1.add_vertex();

    g1.add_edge(v0, v1, 1.0, EdgeType::Directed);
    g1.add_edge(v1, v2, 2.0, EdgeType::Undirected);

    // Copy
    Graph<void> g2(g1);

    SUBCASE("Edge directionality is preserved") {
        CHECK(g2.has_edge(v0, v1) == true);
        CHECK(g2.has_edge(v1, v0) == false); // Directed

        CHECK(g2.has_edge(v1, v2) == true);
        CHECK(g2.has_edge(v2, v1) == true); // Undirected
    }

    SUBCASE("Edge types are preserved in iteration") {
        auto edges = g2.edges();
        CHECK(edges.size() == 2);

        for (const auto &edge : edges) {
            if (edge.source == v0 && edge.target == v1) {
                CHECK(edge.type == EdgeType::Directed);
            } else if ((edge.source == v1 && edge.target == v2) || (edge.source == v2 && edge.target == v1)) {
                CHECK(edge.type == EdgeType::Undirected);
            }
        }
    }
}

TEST_CASE("Graph Copy - Empty graph") {
    Graph<void> g1;

    Graph<void> g2(g1);

    CHECK(g2.vertex_count() == 0);
    CHECK(g2.edge_count() == 0);
}

TEST_CASE("Graph Copy - Large graph") {
    Graph<void> g1;

    // Create a larger graph
    std::vector<typename Graph<void>::VertexId> vertices;
    for (int i = 0; i < 100; ++i) {
        vertices.push_back(g1.add_vertex());
    }

    // Add edges
    for (size_t i = 0; i < vertices.size() - 1; ++i) {
        g1.add_edge(vertices[i], vertices[i + 1], static_cast<double>(i));
    }

    // Copy
    Graph<void> g2(g1);

    CHECK(g2.vertex_count() == 100);
    CHECK(g2.edge_count() == 99);

    // Verify some edges
    auto edge_opt = g2.get_edge(vertices[0], vertices[1]);
    CHECK(edge_opt.has_value());
    CHECK(g2.get_weight(*edge_opt) == 0.0);
}

TEST_CASE("Graph Copy - Self-assignment") {
    Graph<void> g1;

    auto v0 = g1.add_vertex();
    auto v1 = g1.add_vertex();
    g1.add_edge(v0, v1, 1.0);

    // Self-assignment (should be safe)
    g1 = g1;

    CHECK(g1.vertex_count() == 2);
    CHECK(g1.edge_count() == 1);
}

TEST_CASE("Graph Copy - Chain assignment") {
    Graph<void> g1;

    auto v0 = g1.add_vertex();
    auto v1 = g1.add_vertex();
    g1.add_edge(v0, v1, 1.0);

    Graph<void> g2, g3;

    // Chain assignment
    g3 = g2 = g1;

    CHECK(g2.vertex_count() == 2);
    CHECK(g2.edge_count() == 1);
    CHECK(g3.vertex_count() == 2);
    CHECK(g3.edge_count() == 1);
}

TEST_CASE("Graph Move - Move then use moved-to") {
    Graph<void> g1;

    auto v0 = g1.add_vertex();
    auto v1 = g1.add_vertex();
    g1.add_edge(v0, v1, 1.0);

    Graph<void> g2(std::move(g1));

    // Use moved-to graph
    auto v2 = g2.add_vertex();
    g2.add_edge(v1, v2, 2.0);

    CHECK(g2.vertex_count() == 3);
    CHECK(g2.edge_count() == 2);
}
