#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "graphix/vertex/algorithms/graph_properties.hpp"
#include "graphix/vertex/graph.hpp"
#include <cmath>

using namespace graphix::vertex;
using namespace graphix::vertex::algorithms;

TEST_SUITE("Graph Properties - Bipartite") {
    TEST_CASE("Empty graph is bipartite") {
        Graph<void> g;
        auto result = is_bipartite(g);
        CHECK(result.is_bipartite);
        CHECK(result.coloring.empty());
        CHECK(result.odd_cycle.empty());
    }

    TEST_CASE("Single vertex is bipartite") {
        Graph<void> g;
        g.add_vertex();
        auto result = is_bipartite(g);
        CHECK(result.is_bipartite);
        CHECK(result.coloring.size() == 1);
    }

    TEST_CASE("Two vertices with edge is bipartite") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1);

        auto result = is_bipartite(g);
        CHECK(result.is_bipartite);
        CHECK(result.coloring.size() == 2);
        CHECK(result.coloring[v0] != result.coloring[v1]);
    }

    TEST_CASE("Triangle is not bipartite") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1);
        g.add_edge(v1, v2);
        g.add_edge(v2, v0);

        auto result = is_bipartite(g);
        CHECK_FALSE(result.is_bipartite);
        CHECK_FALSE(result.odd_cycle.empty());
    }

    TEST_CASE("Square (4-cycle) is bipartite") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v0, v1);
        g.add_edge(v1, v2);
        g.add_edge(v2, v3);
        g.add_edge(v3, v0);

        auto result = is_bipartite(g);
        CHECK(result.is_bipartite);
        CHECK(result.coloring.size() == 4);
        // Vertices 0,2 should have same color, 1,3 should have same color
        CHECK(result.coloring[v0] == result.coloring[v2]);
        CHECK(result.coloring[v1] == result.coloring[v3]);
        CHECK(result.coloring[v0] != result.coloring[v1]);
    }

    TEST_CASE("Pentagon (5-cycle) is not bipartite") {
        Graph<void> g;
        std::vector<size_t> vertices;
        for (int i = 0; i < 5; i++) {
            vertices.push_back(g.add_vertex());
        }
        for (int i = 0; i < 5; i++) {
            g.add_edge(vertices[i], vertices[(i + 1) % 5]);
        }

        auto result = is_bipartite(g);
        CHECK_FALSE(result.is_bipartite);
        CHECK_FALSE(result.odd_cycle.empty());
    }

    TEST_CASE("Complete bipartite graph K_{2,3}") {
        Graph<void> g;
        auto a0 = g.add_vertex();
        auto a1 = g.add_vertex();
        auto b0 = g.add_vertex();
        auto b1 = g.add_vertex();
        auto b2 = g.add_vertex();

        // Connect all a's to all b's
        for (auto a : {a0, a1}) {
            for (auto b : {b0, b1, b2}) {
                g.add_edge(a, b);
            }
        }

        auto result = is_bipartite(g);
        CHECK(result.is_bipartite);
        CHECK(result.coloring[a0] == result.coloring[a1]);
        CHECK(result.coloring[b0] == result.coloring[b1]);
        CHECK(result.coloring[b0] == result.coloring[b2]);
        CHECK(result.coloring[a0] != result.coloring[b0]);
    }

    TEST_CASE("Disconnected graph with one bipartite and one non-bipartite component") {
        Graph<void> g;
        // Bipartite component: edge
        auto e0 = g.add_vertex();
        auto e1 = g.add_vertex();
        g.add_edge(e0, e1);

        // Non-bipartite component: triangle
        auto t0 = g.add_vertex();
        auto t1 = g.add_vertex();
        auto t2 = g.add_vertex();
        g.add_edge(t0, t1);
        g.add_edge(t1, t2);
        g.add_edge(t2, t0);

        auto result = is_bipartite(g);
        CHECK_FALSE(result.is_bipartite);
    }

    TEST_CASE("Tree is bipartite") {
        Graph<void> g;
        auto root = g.add_vertex();
        auto left = g.add_vertex();
        auto right = g.add_vertex();
        auto ll = g.add_vertex();
        auto lr = g.add_vertex();

        g.add_edge(root, left);
        g.add_edge(root, right);
        g.add_edge(left, ll);
        g.add_edge(left, lr);

        auto result = is_bipartite(g);
        CHECK(result.is_bipartite);
    }

    TEST_CASE("Self-loop makes graph non-bipartite") {
        Graph<void> g;
        auto v = g.add_vertex();
        g.add_edge(v, v);

        auto result = is_bipartite(g);
        CHECK_FALSE(result.is_bipartite);
    }
}

TEST_SUITE("Graph Properties - Acyclic (Undirected)") {
    TEST_CASE("Empty graph is acyclic") {
        Graph<void> g;
        CHECK(is_acyclic(g));
        CHECK(is_acyclic_undirected(g));
    }

    TEST_CASE("Single vertex is acyclic") {
        Graph<void> g;
        g.add_vertex();
        CHECK(is_acyclic(g));
        CHECK(is_acyclic_undirected(g));
    }

    TEST_CASE("Single edge is acyclic") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1);

        CHECK(is_acyclic(g));
        CHECK(is_acyclic_undirected(g));
    }

    TEST_CASE("Tree is acyclic") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v0, v1);
        g.add_edge(v0, v2);
        g.add_edge(v1, v3);

        CHECK(is_acyclic(g));
        CHECK(is_acyclic_undirected(g));
    }

    TEST_CASE("Triangle has cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1);
        g.add_edge(v1, v2);
        g.add_edge(v2, v0);

        CHECK_FALSE(is_acyclic(g));
        CHECK_FALSE(is_acyclic_undirected(g));
    }

    TEST_CASE("4-cycle has cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v0, v1);
        g.add_edge(v1, v2);
        g.add_edge(v2, v3);
        g.add_edge(v3, v0);

        CHECK_FALSE(is_acyclic(g));
        CHECK_FALSE(is_acyclic_undirected(g));
    }

    TEST_CASE("Self-loop creates cycle") {
        Graph<void> g;
        auto v = g.add_vertex();
        g.add_edge(v, v);

        CHECK_FALSE(is_acyclic(g));
        CHECK_FALSE(is_acyclic_undirected(g));
    }

    TEST_CASE("Forest (multiple trees) is acyclic") {
        Graph<void> g;
        // Tree 1
        auto t1_0 = g.add_vertex();
        auto t1_1 = g.add_vertex();
        g.add_edge(t1_0, t1_1);

        // Tree 2
        auto t2_0 = g.add_vertex();
        auto t2_1 = g.add_vertex();
        auto t2_2 = g.add_vertex();
        g.add_edge(t2_0, t2_1);
        g.add_edge(t2_0, t2_2);

        CHECK(is_acyclic(g));
        CHECK(is_acyclic_undirected(g));
    }
}

TEST_SUITE("Graph Properties - Acyclic (Directed)") {
    TEST_CASE("Empty directed graph is acyclic") {
        Graph<void> g;
        CHECK(is_acyclic_directed(g));
    }

    TEST_CASE("Single directed edge is acyclic") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);

        CHECK(is_acyclic_directed(g));
    }

    TEST_CASE("DAG (Directed Acyclic Graph)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        CHECK(is_acyclic_directed(g));
    }

    TEST_CASE("Directed triangle has cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        CHECK_FALSE(is_acyclic_directed(g));
    }

    TEST_CASE("Self-loop in directed graph") {
        Graph<void> g;
        auto v = g.add_vertex();
        g.add_edge(v, v, 1.0, EdgeType::Directed);

        CHECK_FALSE(is_acyclic_directed(g));
    }

    TEST_CASE("Two directed edges forming cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        CHECK_FALSE(is_acyclic_directed(g));
    }

    TEST_CASE("Directed path is acyclic") {
        Graph<void> g;
        std::vector<size_t> vertices;
        for (int i = 0; i < 5; i++) {
            vertices.push_back(g.add_vertex());
        }
        for (int i = 0; i < 4; i++) {
            g.add_edge(vertices[i], vertices[i + 1], 1.0, EdgeType::Directed);
        }

        CHECK(is_acyclic_directed(g));
    }
}

TEST_SUITE("Graph Properties - Diameter, Radius, Center") {
    TEST_CASE("Empty graph diameter") {
        Graph<void> g;
        CHECK(graph_diameter(g) == 0.0);
        CHECK(graph_radius(g) == 0.0);
    }

    TEST_CASE("Single vertex diameter") {
        Graph<void> g;
        g.add_vertex();
        CHECK(graph_diameter(g) == 0.0);
        CHECK(graph_radius(g) == 0.0);
    }

    TEST_CASE("Two vertices with edge") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 2.0);

        CHECK(graph_diameter(g) == 2.0);
        CHECK(graph_radius(g) == 2.0);
        auto center = graph_center(g);
        CHECK(center.size() == 2);
    }

    TEST_CASE("Path graph diameter") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);
        g.add_edge(v2, v3, 1.0);

        // Diameter is distance from v0 to v3 = 3
        CHECK(graph_diameter(g) == 3.0);
        // Radius is min eccentricity (v1 or v2 have eccentricity 2)
        CHECK(graph_radius(g) == 2.0);

        auto center = graph_center(g);
        // Center should be v1 and v2
        CHECK(center.size() == 2);
    }

    TEST_CASE("Triangle diameter") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);
        g.add_edge(v2, v0, 1.0);

        // Triangle: all vertices directly connected, so diameter = 1
        CHECK(graph_diameter(g) == 1.0);
        // All vertices have same eccentricity in triangle
        CHECK(graph_radius(g) == 1.0);
    }

    TEST_CASE("Star graph diameter") {
        Graph<void> g;
        auto center_v = g.add_vertex();
        std::vector<size_t> leaves;
        for (int i = 0; i < 4; i++) {
            leaves.push_back(g.add_vertex());
            g.add_edge(center_v, leaves[i], 1.0);
        }

        // Diameter is distance between two leaves = 2
        CHECK(graph_diameter(g) == 2.0);
        // Radius is eccentricity of center = 1
        CHECK(graph_radius(g) == 1.0);

        auto center = graph_center(g);
        CHECK(center.size() == 1);
        CHECK(center[0] == center_v);
    }

    TEST_CASE("Disconnected graph has infinite diameter") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v0, v1);
        g.add_edge(v2, v3); // Separate component

        CHECK(std::isinf(graph_diameter(g)));
        CHECK(std::isinf(graph_radius(g)));
        CHECK(graph_center(g).empty());
    }

    TEST_CASE("Weighted path diameter") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 5.0);
        g.add_edge(v1, v2, 3.0);

        // Diameter is distance from v0 to v2 = 8.0
        CHECK(graph_diameter(g) == 8.0);
        // Radius is eccentricity of v1 = 5.0
        CHECK(graph_radius(g) == 5.0);

        auto center = graph_center(g);
        CHECK(center.size() == 1);
        CHECK(center[0] == v1);
    }

    TEST_CASE("Complete graph K4 diameter") {
        Graph<void> g;
        std::vector<size_t> vertices;
        for (int i = 0; i < 4; i++) {
            vertices.push_back(g.add_vertex());
        }
        // Add all edges
        for (int i = 0; i < 4; i++) {
            for (int j = i + 1; j < 4; j++) {
                g.add_edge(vertices[i], vertices[j], 1.0);
            }
        }

        // In complete graph, all vertices are adjacent
        CHECK(graph_diameter(g) == 1.0);
        CHECK(graph_radius(g) == 1.0);

        auto center = graph_center(g);
        // All vertices are centers in complete graph
        CHECK(center.size() == 4);
    }
}

TEST_SUITE("Graph Properties - Edge Cases") {
    TEST_CASE("Graph with isolated vertices") {
        Graph<void> g;
        g.add_vertex();
        g.add_vertex();
        g.add_vertex();

        // Disconnected, so not bipartite check fails (but technically each component is)
        auto bip_result = is_bipartite(g);
        CHECK(bip_result.is_bipartite); // Isolated vertices are bipartite

        CHECK(is_acyclic(g)); // No edges, so acyclic

        CHECK(std::isinf(graph_diameter(g))); // Disconnected
    }

    TEST_CASE("Bipartite with multiple components") {
        Graph<void> g;
        // Component 1: K_{2,2}
        auto a0 = g.add_vertex();
        auto a1 = g.add_vertex();
        auto b0 = g.add_vertex();
        auto b1 = g.add_vertex();
        g.add_edge(a0, b0);
        g.add_edge(a0, b1);
        g.add_edge(a1, b0);
        g.add_edge(a1, b1);

        // Component 2: Single edge
        auto c0 = g.add_vertex();
        auto c1 = g.add_vertex();
        g.add_edge(c0, c1);

        auto result = is_bipartite(g);
        CHECK(result.is_bipartite);
    }
}
