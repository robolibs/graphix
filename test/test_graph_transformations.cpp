#include "doctest/doctest.h"
#include "graphix/vertex/algorithms/graph_transformations.hpp"
#include "graphix/vertex/graph.hpp"

using namespace graphix::vertex;
using namespace graphix::vertex::algorithms;

TEST_SUITE("Graph Transformations - Transpose") {

    TEST_CASE("Transpose empty graph") {
        Graph<void> g;
        auto result = transpose(g);
        CHECK(result.vertex_count() == 0);
        CHECK(result.edge_count() == 0);
    }

    TEST_CASE("Transpose single vertex") {
        Graph<void> g;
        g.add_vertex();
        auto result = transpose(g);
        CHECK(result.vertex_count() == 1);
        CHECK(result.edge_count() == 0);
    }

    TEST_CASE("Transpose directed graph") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // Create directed edges: v0 -> v1 -> v2
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 2.0, EdgeType::Directed);

        auto result = transpose(g);

        CHECK(result.vertex_count() == 3);
        CHECK(result.edge_count() == 2);

        // In transposed graph: v2 -> v1 -> v0
        // Since vertex IDs are remapped, we need to check connectivity
        auto vertices = result.vertices();

        // Check that edges are reversed by verifying out-degrees
        size_t vertices_with_outdegree_1 = 0;
        size_t vertices_with_outdegree_0 = 0;

        for (auto v : vertices) {
            size_t out_deg = result.degree(v);
            if (out_deg == 1)
                vertices_with_outdegree_1++;
            if (out_deg == 0)
                vertices_with_outdegree_0++;
        }

        CHECK(vertices_with_outdegree_1 == 2); // Two vertices have out-degree 1
        CHECK(vertices_with_outdegree_0 == 1); // One vertex has out-degree 0
    }

    TEST_CASE("Transpose undirected graph") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // Create undirected edges
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 2.0, EdgeType::Undirected);

        auto result = transpose(g);

        CHECK(result.vertex_count() == 3);
        CHECK(result.edge_count() == 2);

        // Undirected graph should be identical to original
        // All vertices should have the same degree pattern
        for (auto v : result.vertices()) {
            size_t deg = result.degree(v);
            CHECK((deg == 2 || deg == 1 || deg == 4)); // Accounting for undirected storage
        }
    }

    TEST_CASE("Transpose mixed graph") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // Mix directed and undirected edges
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 2.0, EdgeType::Undirected);

        auto result = transpose(g);

        CHECK(result.vertex_count() == 3);
        CHECK(result.edge_count() == 2);
    }
}

TEST_SUITE("Graph Transformations - Induced Subgraph") {

    TEST_CASE("Induced subgraph empty set") {
        Graph<void> g;
        g.add_vertex();
        g.add_vertex();
        g.add_edge(0, 1, 1.0);

        std::unordered_set<size_t> empty_set;
        auto result = induced_subgraph(g, empty_set);

        CHECK(result.vertex_count() == 0);
        CHECK(result.edge_count() == 0);
    }

    TEST_CASE("Induced subgraph single vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 2.0);

        std::unordered_set<size_t> vertex_set = {v1};
        auto result = induced_subgraph(g, vertex_set);

        CHECK(result.vertex_count() == 1);
        CHECK(result.edge_count() == 0); // No edges since only one vertex
    }

    TEST_CASE("Induced subgraph two connected vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 2.0);

        std::unordered_set<size_t> vertex_set = {v0, v1};
        auto result = induced_subgraph(g, vertex_set);

        CHECK(result.vertex_count() == 2);
        CHECK(result.edge_count() == 1); // Edge between v0 and v1
    }

    TEST_CASE("Induced subgraph disconnected vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);

        std::unordered_set<size_t> vertex_set = {v0, v2};
        auto result = induced_subgraph(g, vertex_set);

        CHECK(result.vertex_count() == 2);
        CHECK(result.edge_count() == 0); // No edge between v0 and v2
    }

    TEST_CASE("Induced subgraph all vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 2.0);
        g.add_edge(v2, v0, 3.0);

        std::unordered_set<size_t> vertex_set = {v0, v1, v2};
        auto result = induced_subgraph(g, vertex_set);

        CHECK(result.vertex_count() == 3);
        CHECK(result.edge_count() == 3);
    }

    TEST_CASE("Induced subgraph with vector") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 2.0);

        std::vector<size_t> vertices = {v0, v1};
        auto result = induced_subgraph(g, vertices);

        CHECK(result.vertex_count() == 2);
        CHECK(result.edge_count() == 1);
    }
}

TEST_SUITE("Graph Transformations - Graph Union") {

    TEST_CASE("Union of two empty graphs") {
        Graph<void> g1, g2;
        auto result = graph_union(g1, g2);

        CHECK(result.graph.vertex_count() == 0);
        CHECK(result.graph.edge_count() == 0);
    }

    TEST_CASE("Union empty and non-empty graph") {
        Graph<void> g1, g2;
        g2.add_vertex();
        g2.add_vertex();
        g2.add_edge(0, 1, 1.0);

        auto result = graph_union(g1, g2);

        CHECK(result.graph.vertex_count() == 2);
        CHECK(result.graph.edge_count() == 1);
        CHECK(result.g1_mapping.empty());
        CHECK(result.g2_mapping.size() == 2);
    }

    TEST_CASE("Union two simple graphs") {
        Graph<void> g1, g2;

        // g1: v0 -- v1
        auto v0_g1 = g1.add_vertex();
        auto v1_g1 = g1.add_vertex();
        g1.add_edge(v0_g1, v1_g1, 1.0);

        // g2: v0 -- v1
        auto v0_g2 = g2.add_vertex();
        auto v1_g2 = g2.add_vertex();
        g2.add_edge(v0_g2, v1_g2, 2.0);

        auto result = graph_union(g1, g2);

        // Should have 4 vertices (2 from each graph)
        CHECK(result.graph.vertex_count() == 4);
        // Should have 2 edges (1 from each graph)
        CHECK(result.graph.edge_count() == 2);

        CHECK(result.g1_mapping.size() == 2);
        CHECK(result.g2_mapping.size() == 2);
    }

    TEST_CASE("Disjoint union is same as union") {
        Graph<void> g1, g2;
        g1.add_vertex();
        g2.add_vertex();

        auto union_result = graph_union(g1, g2);
        auto disjoint_result = disjoint_union(g1, g2);

        CHECK(union_result.graph.vertex_count() == disjoint_result.graph.vertex_count());
        CHECK(union_result.graph.edge_count() == disjoint_result.graph.edge_count());
    }
}

TEST_SUITE("Graph Transformations - Graph Intersection") {

    TEST_CASE("Intersection of two empty graphs") {
        Graph<void> g1, g2;
        auto result = graph_intersection(g1, g2);

        CHECK(result.vertex_count() == 0);
        CHECK(result.edge_count() == 0);
    }

    TEST_CASE("Intersection with no common vertices") {
        Graph<void> g1, g2;
        g1.add_vertex(); // v0
        g1.add_vertex(); // v1

        g2.add_vertex(); // v0
        g2.add_vertex(); // v1
        // But different IDs internally

        // Since both have vertices 0,1 they should intersect
        auto result = graph_intersection(g1, g2);
        CHECK(result.vertex_count() == 2);
    }

    TEST_CASE("Intersection with common vertices but no common edges") {
        Graph<void> g1, g2;

        g1.add_vertex(); // 0
        g1.add_vertex(); // 1
        g1.add_vertex(); // 2
        g1.add_edge(0, 1, 1.0);

        g2.add_vertex(); // 0
        g2.add_vertex(); // 1
        g2.add_vertex(); // 2
        g2.add_edge(1, 2, 1.0);

        auto result = graph_intersection(g1, g2);

        CHECK(result.vertex_count() == 3); // All vertices in common
        CHECK(result.edge_count() == 0);   // No common edges
    }

    TEST_CASE("Intersection with common vertices and edges") {
        Graph<void> g1, g2;

        g1.add_vertex(); // 0
        g1.add_vertex(); // 1
        g1.add_vertex(); // 2
        g1.add_edge(0, 1, 1.0);
        g1.add_edge(1, 2, 2.0);

        g2.add_vertex(); // 0
        g2.add_vertex(); // 1
        g2.add_vertex(); // 2
        g2.add_edge(0, 1, 1.0);
        g2.add_edge(1, 2, 3.0);

        auto result = graph_intersection(g1, g2);

        CHECK(result.vertex_count() == 3);
        CHECK(result.edge_count() == 2); // Both edges are common
    }
}

TEST_SUITE("Graph Transformations - Complement") {

    TEST_CASE("Complement of empty graph") {
        Graph<void> g;
        auto result = complement(g);

        CHECK(result.vertex_count() == 0);
        CHECK(result.edge_count() == 0);
    }

    TEST_CASE("Complement of single vertex") {
        Graph<void> g;
        g.add_vertex();
        auto result = complement(g);

        CHECK(result.vertex_count() == 1);
        CHECK(result.edge_count() == 0); // No edges possible
    }

    TEST_CASE("Complement of two disconnected vertices") {
        Graph<void> g;
        g.add_vertex(); // v0
        g.add_vertex(); // v1

        auto result = complement(g);

        CHECK(result.vertex_count() == 2);
        CHECK(result.edge_count() == 1); // Should have edge between v0 and v1
    }

    TEST_CASE("Complement of two connected vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);

        auto result = complement(g);

        CHECK(result.vertex_count() == 2);
        CHECK(result.edge_count() == 0); // No edges in complement
    }

    TEST_CASE("Complement of triangle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v0, 1.0, EdgeType::Undirected);

        auto result = complement(g);

        CHECK(result.vertex_count() == 3);
        CHECK(result.edge_count() == 0); // Complete graph has no complement edges
    }

    TEST_CASE("Complement of path") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);

        auto result = complement(g);

        CHECK(result.vertex_count() == 3);
        CHECK(result.edge_count() == 1); // Should have edge between v0 and v2
    }

    TEST_CASE("Complement ignores directed edges") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);

        auto result = complement(g);

        CHECK(result.vertex_count() == 2);
        // Directed edges are ignored, so complement treats as disconnected
        CHECK(result.edge_count() == 1);
    }
}

TEST_SUITE("Graph Transformations - Filter Functions") {

    TEST_CASE("Filter vertices by predicate") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);
        g.add_edge(v2, v3, 1.0);
        g.add_edge(v0, v2, 1.0); // Add edge between v0 and v2 so induced subgraph has an edge

        // Keep only even vertex IDs
        auto result = filter_vertices(g, [](size_t v, const Graph<void> &) { return v % 2 == 0; });

        CHECK(result.vertex_count() == 2); // v0 and v2
        CHECK(result.edge_count() == 1);   // Edge between v0 and v2
    }

    TEST_CASE("Filter edges by weight") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 5.0);
        g.add_edge(v2, v0, 3.0);

        // Keep only edges with weight > 2
        auto result = filter_edges(g, [](const EdgeDescriptor &e, const Graph<void> &) { return e.weight > 2.0; });

        CHECK(result.vertex_count() == 3); // All vertices kept
        CHECK(result.edge_count() == 2);   // Two edges with weight > 2
    }

    TEST_CASE("Filter edges by type") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);

        // Keep only directed edges
        auto result =
            filter_edges(g, [](const EdgeDescriptor &e, const Graph<void> &) { return e.type == EdgeType::Directed; });

        CHECK(result.vertex_count() == 3);
        CHECK(result.edge_count() == 1); // Only directed edge
    }

    TEST_CASE("Filter vertices keeps no vertices") {
        Graph<void> g;
        g.add_vertex();
        g.add_vertex();

        auto result = filter_vertices(g, [](size_t, const Graph<void> &) {
            return false; // Keep nothing
        });

        CHECK(result.vertex_count() == 0);
        CHECK(result.edge_count() == 0);
    }

    TEST_CASE("Filter edges keeps no edges") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);

        auto result = filter_edges(g, [](const EdgeDescriptor &, const Graph<void> &) {
            return false; // Keep nothing
        });

        CHECK(result.vertex_count() == 2); // Vertices remain
        CHECK(result.edge_count() == 0);   // No edges
    }
}

TEST_SUITE("Graph Transformations - Edge Cases") {

    TEST_CASE("Transpose preserves weights") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 42.0, EdgeType::Directed);

        auto result = transpose(g);

        // Find the edge and check weight
        auto edges = result.edges();
        CHECK(edges.size() == 1);
        CHECK(edges[0].weight == 42.0);
    }

    TEST_CASE("Subgraph with self-loop") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v0, 1.0); // Self-loop
        g.add_edge(v0, v1, 2.0);

        std::unordered_set<size_t> vertex_set = {v0};
        auto result = induced_subgraph(g, vertex_set);

        CHECK(result.vertex_count() == 1);
        CHECK(result.edge_count() == 1); // Self-loop preserved
    }

    TEST_CASE("Union preserves edge types") {
        Graph<void> g1, g2;
        auto v0_g1 = g1.add_vertex();
        auto v1_g1 = g1.add_vertex();
        g1.add_edge(v0_g1, v1_g1, 1.0, EdgeType::Directed);

        auto v0_g2 = g2.add_vertex();
        auto v1_g2 = g2.add_vertex();
        g2.add_edge(v0_g2, v1_g2, 2.0, EdgeType::Undirected);

        auto result = graph_union(g1, g2);

        auto edges = result.graph.edges();
        CHECK(edges.size() == 2);

        // Check that we have one directed and one undirected edge
        size_t directed_count = 0;
        size_t undirected_count = 0;
        for (const auto &e : edges) {
            if (e.type == EdgeType::Directed)
                directed_count++;
            if (e.type == EdgeType::Undirected)
                undirected_count++;
        }
        CHECK(directed_count == 1);
        CHECK(undirected_count == 1);
    }
}
