#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"
#include "graphix/vertex/algorithms/centrality.hpp"
#include "graphix/vertex/graph.hpp"

using namespace graphix::vertex;
using namespace graphix::vertex::algorithms;

TEST_SUITE("Centrality - Degree Centrality") {

    TEST_CASE("Empty graph") {
        Graph<void> g;
        auto centrality = degree_centrality_all(g);
        CHECK(centrality.empty());
    }

    TEST_CASE("Single vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();

        CHECK(degree_centrality(g, v0) == 0.0);

        auto centrality = degree_centrality_all(g);
        CHECK(centrality[v0] == 0.0);
    }

    TEST_CASE("Two vertices, one edge (undirected)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);

        // Each vertex has degree 1, max possible is 1
        // For undirected: degree / (2 * (n-1)) = 1 / (2 * 1) = 0.5
        CHECK(degree_centrality(g, v0) == doctest::Approx(0.5));
        CHECK(degree_centrality(g, v1) == doctest::Approx(0.5));
    }

    TEST_CASE("Triangle (undirected)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1);
        g.add_edge(v1, v2);
        g.add_edge(v2, v0);

        // All vertices have degree 2, max possible is 2
        // For undirected: 2 / (2 * 2) = 0.5
        auto centrality = degree_centrality_all(g);
        CHECK(centrality[v0] == doctest::Approx(0.5));
        CHECK(centrality[v1] == doctest::Approx(0.5));
        CHECK(centrality[v2] == doctest::Approx(0.5));
    }

    TEST_CASE("Star graph (undirected)") {
        Graph<void> g;
        auto center = g.add_vertex();
        std::vector<size_t> leaves;
        for (int i = 0; i < 4; i++) {
            leaves.push_back(g.add_vertex());
            g.add_edge(center, leaves[i]);
        }

        // Center has degree 4, leaves have degree 1
        // n = 5, so max = 4
        // Center: 4 / (2 * 4) = 0.5
        // Leaves: 1 / (2 * 4) = 0.125
        CHECK(degree_centrality(g, center) == doctest::Approx(0.5));
        for (auto leaf : leaves) {
            CHECK(degree_centrality(g, leaf) == doctest::Approx(0.125));
        }
    }

    TEST_CASE("Directed graph") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // v0 -> v1 -> v2
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        // For directed: degree / (n-1)
        // v0: out-degree=1, so 1/2 = 0.5
        // v1: out-degree=1, in-degree=1, total=2, so 2/2 = 1.0
        // v2: in-degree=1, so 1/2 = 0.5
        CHECK(degree_centrality(g, v0) == doctest::Approx(0.5));
        CHECK(degree_centrality(g, v1) == doctest::Approx(1.0));
        CHECK(degree_centrality(g, v2) == doctest::Approx(0.5));
    }
}

TEST_SUITE("Centrality - Closeness Centrality") {

    TEST_CASE("Empty graph") {
        Graph<void> g;
        auto centrality = closeness_centrality_all(g);
        CHECK(centrality.empty());
    }

    TEST_CASE("Single vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();

        CHECK(closeness_centrality(g, v0) == 0.0);
    }

    TEST_CASE("Two vertices connected") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);

        // Distance from v0 to v1 is 1
        // Closeness = (n-1) / sum_distances = 1 / 1 = 1.0
        CHECK(closeness_centrality(g, v0) == doctest::Approx(1.0));
        CHECK(closeness_centrality(g, v1) == doctest::Approx(1.0));
    }

    TEST_CASE("Path graph (3 vertices)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);

        // v0: distances [0, 1, 2] sum=3, closeness = 2/3 ≈ 0.667
        // v1: distances [1, 0, 1] sum=2, closeness = 2/2 = 1.0
        // v2: distances [2, 1, 0] sum=3, closeness = 2/3 ≈ 0.667
        CHECK(closeness_centrality(g, v0) == doctest::Approx(2.0 / 3.0));
        CHECK(closeness_centrality(g, v1) == doctest::Approx(1.0));
        CHECK(closeness_centrality(g, v2) == doctest::Approx(2.0 / 3.0));
    }

    TEST_CASE("Star graph") {
        Graph<void> g;
        auto center = g.add_vertex();
        std::vector<size_t> leaves;
        for (int i = 0; i < 3; i++) {
            leaves.push_back(g.add_vertex());
            g.add_edge(center, leaves[i], 1.0);
        }

        // Center: distances to all leaves = 1, sum = 3, closeness = 3/3 = 1.0
        // Leaf: distances [1, 2, 2], sum = 5, closeness = 3/5 = 0.6
        CHECK(closeness_centrality(g, center) == doctest::Approx(1.0));
        for (auto leaf : leaves) {
            CHECK(closeness_centrality(g, leaf) == doctest::Approx(3.0 / 5.0));
        }
    }

    TEST_CASE("Disconnected graph") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Two components: {v0, v1} and {v2, v3}
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v2, v3, 1.0);

        // Cannot reach all vertices, should return 0
        CHECK(closeness_centrality(g, v0) == 0.0);
        CHECK(closeness_centrality(g, v2) == 0.0);
    }

    TEST_CASE("Triangle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);
        g.add_edge(v2, v0, 1.0);

        // All vertices same distance to others: [0, 1, 1], sum = 2
        // Closeness = 2/2 = 1.0
        CHECK(closeness_centrality(g, v0) == doctest::Approx(1.0));
        CHECK(closeness_centrality(g, v1) == doctest::Approx(1.0));
        CHECK(closeness_centrality(g, v2) == doctest::Approx(1.0));
    }
}

TEST_SUITE("Centrality - Betweenness Centrality") {

    TEST_CASE("Empty graph") {
        Graph<void> g;
        auto centrality = betweenness_centrality(g);
        CHECK(centrality.empty());
    }

    TEST_CASE("Single vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();

        auto centrality = betweenness_centrality(g);
        CHECK(centrality[v0] == 0.0);
    }

    TEST_CASE("Two vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);

        auto centrality = betweenness_centrality(g);
        // No vertex lies between others
        CHECK(centrality[v0] == 0.0);
        CHECK(centrality[v1] == 0.0);
    }

    TEST_CASE("Path graph (3 vertices)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);

        auto centrality = betweenness_centrality(g);

        // v1 is on the path between v0 and v2
        // For undirected: each path counted once in each direction
        // v0-v2 path goes through v1: contributes 1
        // v2-v0 path goes through v1: contributes 1
        // Total for v1: 2, but normalized by 0.5 for undirected = 1
        CHECK(centrality[v0] == 0.0);
        CHECK(centrality[v1] == doctest::Approx(1.0));
        CHECK(centrality[v2] == 0.0);
    }

    TEST_CASE("Path graph (4 vertices)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);
        g.add_edge(v2, v3, 1.0);

        auto centrality = betweenness_centrality(g);

        // v1 lies on paths: v0-v2, v0-v3
        // v2 lies on paths: v0-v3, v1-v3
        // For undirected, each counted in both directions
        CHECK(centrality[v0] == 0.0);
        CHECK(centrality[v1] > centrality[v0]);
        CHECK(centrality[v2] > centrality[v0]);
        CHECK(centrality[v3] == 0.0);
    }

    TEST_CASE("Star graph") {
        Graph<void> g;
        auto center = g.add_vertex();
        std::vector<size_t> leaves;
        for (int i = 0; i < 4; i++) {
            leaves.push_back(g.add_vertex());
            g.add_edge(center, leaves[i], 1.0);
        }

        auto centrality = betweenness_centrality(g);

        // Center is on ALL paths between leaves
        // With 4 leaves, there are C(4,2) = 6 pairs of leaves
        // Each pair's shortest path goes through center
        CHECK(centrality[center] > 0.0);
        for (auto leaf : leaves) {
            CHECK(centrality[leaf] == 0.0);
        }
    }

    TEST_CASE("Triangle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);
        g.add_edge(v2, v0, 1.0);

        auto centrality = betweenness_centrality(g);

        // All vertices equivalent in triangle, no vertex lies on shortest paths
        CHECK(centrality[v0] == doctest::Approx(0.0));
        CHECK(centrality[v1] == doctest::Approx(0.0));
        CHECK(centrality[v2] == doctest::Approx(0.0));
    }

    TEST_CASE("Diamond graph") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Diamond: 0 connects to 1,2; both connect to 3
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v0, v2, 1.0);
        g.add_edge(v1, v3, 1.0);
        g.add_edge(v2, v3, 1.0);

        auto centrality = betweenness_centrality(g);

        // v0 and v3 have some betweenness
        // v1 and v2 share paths from v0 to v3
        CHECK(centrality[v0] >= 0.0);
        CHECK(centrality[v1] >= 0.0);
        CHECK(centrality[v2] >= 0.0);
        CHECK(centrality[v3] >= 0.0);
    }

    TEST_CASE("Directed path") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        auto centrality = betweenness_centrality(g);

        // v1 is on the directed path from v0 to v2
        CHECK(centrality[v0] == 0.0);
        CHECK(centrality[v1] == doctest::Approx(1.0)); // For directed, no division by 2
        CHECK(centrality[v2] == 0.0);
    }
}

TEST_SUITE("Centrality - Normalized Betweenness") {

    TEST_CASE("Path graph normalized") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);

        auto centrality = betweenness_centrality_normalized(g);

        // With n=3, normalization factor = (n-1)*(n-2) = 2*1 = 2
        // v1 has betweenness 1.0, normalized = 1.0 / 2 = 0.5
        CHECK(centrality[v0] == 0.0);
        CHECK(centrality[v1] == doctest::Approx(0.5));
        CHECK(centrality[v2] == 0.0);
    }

    TEST_CASE("Complete graph K4 normalized") {
        Graph<void> g;
        std::vector<size_t> vertices;
        for (int i = 0; i < 4; i++) {
            vertices.push_back(g.add_vertex());
        }

        // Create complete graph
        for (size_t i = 0; i < vertices.size(); i++) {
            for (size_t j = i + 1; j < vertices.size(); j++) {
                g.add_edge(vertices[i], vertices[j], 1.0);
            }
        }

        auto centrality = betweenness_centrality_normalized(g);

        // In complete graph, all shortest paths are direct (length 1)
        // No vertex lies on any shortest path
        for (auto v : vertices) {
            CHECK(centrality[v] == doctest::Approx(0.0));
        }
    }
}

TEST_SUITE("Centrality - Utility Functions") {

    TEST_CASE("top_k_central_vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Star graph: center is v1
        g.add_edge(v1, v0, 1.0);
        g.add_edge(v1, v2, 1.0);
        g.add_edge(v1, v3, 1.0);

        auto centrality = degree_centrality_all(g);
        auto top_1 = top_k_central_vertices(centrality, 1);
        auto top_2 = top_k_central_vertices(centrality, 2);

        CHECK(top_1.size() == 1);
        CHECK(top_1[0] == v1); // Center has highest degree centrality

        CHECK(top_2.size() == 2);
        CHECK(top_2[0] == v1); // Center first
    }

    TEST_CASE("most_central_vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Star graph
        g.add_edge(v1, v0, 1.0);
        g.add_edge(v1, v2, 1.0);
        g.add_edge(v1, v3, 1.0);

        auto degree_cent = degree_centrality_all(g);
        CHECK(most_central_vertex(degree_cent) == v1);

        auto closeness_cent = closeness_centrality_all(g);
        CHECK(most_central_vertex(closeness_cent) == v1);
    }

    TEST_CASE("most_central_vertex on empty map throws") {
        std::unordered_map<size_t, double> empty;
        CHECK_THROWS_AS(most_central_vertex(empty), std::invalid_argument);
    }
}

TEST_SUITE("Centrality - Edge Cases") {

    TEST_CASE("Self-loop") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v0, 1.0); // Self-loop
        g.add_edge(v0, v1, 1.0);

        // Should handle self-loops gracefully
        CHECK_NOTHROW(degree_centrality(g, v0));
        CHECK_NOTHROW(closeness_centrality(g, v0));
    }

    TEST_CASE("Isolated vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0);
        // v2 is isolated

        CHECK(degree_centrality(g, v2) == 0.0);
        CHECK(closeness_centrality(g, v2) == 0.0);
    }

    TEST_CASE("Weighted edges (closeness)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 10.0);
        g.add_edge(v1, v2, 1.0);

        // v0: distances [0, 10, 11], sum=21, closeness = 2/21
        // v1: distances [10, 0, 1], sum=11, closeness = 2/11
        // v2: distances [11, 1, 0], sum=12, closeness = 2/12
        CHECK(closeness_centrality(g, v1) > closeness_centrality(g, v0));
        CHECK(closeness_centrality(g, v1) > closeness_centrality(g, v2));
    }

    TEST_CASE("Invalid vertex throws") {
        Graph<void> g;
        auto v0 = g.add_vertex();

        CHECK_THROWS_AS(degree_centrality(g, 999), std::invalid_argument);
        CHECK_THROWS_AS(closeness_centrality(g, 999), std::invalid_argument);
    }
}
