#include "graphix/vertex/algorithms/strongly_connected_components.hpp"
#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

using namespace graphix::vertex;

TEST_SUITE("Strongly Connected Components - Basic") {
    TEST_CASE("Empty graph") {
        Graph<void> g;
        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 0);
        CHECK(result.components.empty());
    }

    TEST_CASE("Single vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto result = strongly_connected_components(g);

        CHECK(result.num_components == 1);
        REQUIRE(result.components.size() == 1);
        CHECK(result.components[0].size() == 1);
        CHECK(result.components[0][0] == v0);
    }

    TEST_CASE("Two isolated vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 2);
        REQUIRE(result.components.size() == 2);
    }

    TEST_CASE("Two vertices with directed edge (not strongly connected)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 2);
        REQUIRE(result.components.size() == 2);

        // Each vertex is its own SCC
        CHECK(result.components[0].size() == 1);
        CHECK(result.components[1].size() == 1);
    }

    TEST_CASE("Two vertices with bidirectional edges (strongly connected)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 1);
        REQUIRE(result.components.size() == 1);
        CHECK(result.components[0].size() == 2);
    }

    TEST_CASE("Self-loop creates single-vertex SCC") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        g.add_edge(v0, v0, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 1);
        REQUIRE(result.components.size() == 1);
        CHECK(result.components[0].size() == 1);
    }
}

TEST_SUITE("Strongly Connected Components - Cycles") {
    TEST_CASE("Triangle cycle: 0 -> 1 -> 2 -> 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 1);
        REQUIRE(result.components.size() == 1);
        CHECK(result.components[0].size() == 3);

        // All vertices should be in the same component
        CHECK(std::find(result.components[0].begin(), result.components[0].end(), v0) != result.components[0].end());
        CHECK(std::find(result.components[0].begin(), result.components[0].end(), v1) != result.components[0].end());
        CHECK(std::find(result.components[0].begin(), result.components[0].end(), v2) != result.components[0].end());
    }

    TEST_CASE("Large cycle") {
        Graph<void> g;
        std::vector<size_t> vertices;

        for (int i = 0; i < 10; i++) {
            vertices.push_back(g.add_vertex());
        }

        // Create cycle: 0 -> 1 -> ... -> 9 -> 0
        for (int i = 0; i < 10; i++) {
            g.add_edge(vertices[i], vertices[(i + 1) % 10], 1.0, EdgeType::Directed);
        }

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 1);
        REQUIRE(result.components.size() == 1);
        CHECK(result.components[0].size() == 10);
    }

    TEST_CASE("Multiple separate cycles") {
        Graph<void> g;

        // Cycle 1: 0 -> 1 -> 0
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        // Cycle 2: 2 -> 3 -> 4 -> 2
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        auto v4 = g.add_vertex();
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);
        g.add_edge(v3, v4, 1.0, EdgeType::Directed);
        g.add_edge(v4, v2, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 2);
        REQUIRE(result.components.size() == 2);

        // One component should have 2 vertices, the other 3
        bool has_size_2 = false, has_size_3 = false;
        for (const auto &comp : result.components) {
            if (comp.size() == 2)
                has_size_2 = true;
            if (comp.size() == 3)
                has_size_3 = true;
        }
        CHECK(has_size_2);
        CHECK(has_size_3);
    }
}

TEST_SUITE("Strongly Connected Components - DAGs") {
    TEST_CASE("Linear chain (DAG)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 4);
        REQUIRE(result.components.size() == 4);

        // Each vertex is its own SCC in a DAG
        for (const auto &comp : result.components) {
            CHECK(comp.size() == 1);
        }
    }

    TEST_CASE("Diamond DAG") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 4);

        // Each vertex is its own SCC
        for (const auto &comp : result.components) {
            CHECK(comp.size() == 1);
        }
    }
}

TEST_SUITE("Strongly Connected Components - Mixed Structures") {
    TEST_CASE("DAG with embedded cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        auto v4 = g.add_vertex();

        // Path: 0 -> 1
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);

        // Cycle: 1 -> 2 -> 3 -> 1
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);
        g.add_edge(v3, v1, 1.0, EdgeType::Directed);

        // Path: 3 -> 4
        g.add_edge(v3, v4, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 3);
        REQUIRE(result.components.size() == 3);

        // One component should have 3 vertices (the cycle)
        bool has_cycle = false;
        for (const auto &comp : result.components) {
            if (comp.size() == 3) {
                has_cycle = true;
                // Verify it's vertices 1, 2, 3
                CHECK(std::find(comp.begin(), comp.end(), v1) != comp.end());
                CHECK(std::find(comp.begin(), comp.end(), v2) != comp.end());
                CHECK(std::find(comp.begin(), comp.end(), v3) != comp.end());
            }
        }
        CHECK(has_cycle);
    }

    TEST_CASE("Multiple cycles connected") {
        Graph<void> g;

        // Cycle 1: 0 -> 1 -> 0
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        // Cycle 2: 2 -> 3 -> 2
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);
        g.add_edge(v3, v2, 1.0, EdgeType::Directed);

        // Connect cycles: 1 -> 2 (one-way)
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 2);

        // Both components should have 2 vertices each
        for (const auto &comp : result.components) {
            CHECK(comp.size() == 2);
        }
    }

    TEST_CASE("Complex graph with multiple SCCs") {
        Graph<void> g;

        // Create complex structure with 3 SCCs
        // SCC 1: 0 <-> 1
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        // SCC 2: 2 <-> 3 <-> 4 (all interconnected)
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        auto v4 = g.add_vertex();
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);
        g.add_edge(v3, v4, 1.0, EdgeType::Directed);
        g.add_edge(v4, v2, 1.0, EdgeType::Directed);

        // SCC 3: just vertex 5 (isolated)
        auto v5 = g.add_vertex();

        // Connections between SCCs (one-way)
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v4, v5, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);
        CHECK(result.num_components == 3);
        REQUIRE(result.components.size() == 3);

        // Check sizes: 2, 3, 1
        std::vector<size_t> sizes;
        for (const auto &comp : result.components) {
            sizes.push_back(comp.size());
        }
        std::sort(sizes.begin(), sizes.end());
        CHECK(sizes[0] == 1);
        CHECK(sizes[1] == 2);
        CHECK(sizes[2] == 3);
    }
}

TEST_SUITE("Strongly Connected Components - Helper Functions") {
    TEST_CASE("is_strongly_connected - single SCC") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        CHECK(is_strongly_connected(g));
    }

    TEST_CASE("is_strongly_connected - multiple SCCs") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        CHECK_FALSE(is_strongly_connected(g));
    }

    TEST_CASE("get_component_map") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Two SCCs: {0, 1} and {2, 3}
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);
        g.add_edge(v3, v2, 1.0, EdgeType::Directed);

        auto comp_map = get_component_map(g);

        CHECK(comp_map.size() == 4);
        CHECK(comp_map[v0] == comp_map[v1]);
        CHECK(comp_map[v2] == comp_map[v3]);
        CHECK(comp_map[v0] != comp_map[v2]);
    }

    TEST_CASE("largest_scc_size") {
        Graph<void> g;

        // Small SCC: 0 <-> 1
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        // Large SCC: 2 <-> 3 <-> 4 <-> 5 (cycle)
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        auto v4 = g.add_vertex();
        auto v5 = g.add_vertex();
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);
        g.add_edge(v3, v4, 1.0, EdgeType::Directed);
        g.add_edge(v4, v5, 1.0, EdgeType::Directed);
        g.add_edge(v5, v2, 1.0, EdgeType::Directed);

        CHECK(largest_scc_size(g) == 4);
    }

    TEST_CASE("in_same_scc - true case") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        CHECK(in_same_scc(g, v0, v1));
        CHECK(in_same_scc(g, v1, v2));
        CHECK(in_same_scc(g, v0, v2));
    }

    TEST_CASE("in_same_scc - false case") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        CHECK_FALSE(in_same_scc(g, v0, v2));
    }
}

TEST_SUITE("Strongly Connected Components - Edge Cases") {
    TEST_CASE("Undirected edges are ignored") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // Only undirected edges
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);

        auto result = strongly_connected_components(g);

        // Without directed edges, each vertex is its own SCC
        CHECK(result.num_components == 3);
    }

    TEST_CASE("Mixed directed and undirected edges") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // Directed cycle
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        // Undirected connection (should be ignored)
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);

        auto result = strongly_connected_components(g);

        // v0 and v1 form one SCC, v2 is separate
        CHECK(result.num_components == 2);
    }

    TEST_CASE("Large graph performance") {
        Graph<void> g;
        std::vector<size_t> vertices;

        // Create 100 vertices
        for (int i = 0; i < 100; i++) {
            vertices.push_back(g.add_vertex());
        }

        // Create 10 SCCs of 10 vertices each
        for (int scc = 0; scc < 10; scc++) {
            int start = scc * 10;
            for (int i = 0; i < 10; i++) {
                int next = (i + 1) % 10;
                g.add_edge(vertices[start + i], vertices[start + next], 1.0, EdgeType::Directed);
            }
        }

        auto result = strongly_connected_components(g);

        CHECK(result.num_components == 10);
        for (const auto &comp : result.components) {
            CHECK(comp.size() == 10);
        }
    }
}

TEST_SUITE("Strongly Connected Components - With Properties") {
    TEST_CASE("Graph with string properties") {
        Graph<std::string> g;

        auto v0 = g.add_vertex("A");
        auto v1 = g.add_vertex("B");
        auto v2 = g.add_vertex("C");

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);

        CHECK(result.num_components == 1);
        REQUIRE(result.components.size() == 1);
        CHECK(result.components[0].size() == 3);

        // Verify properties are preserved
        for (auto v : result.components[0]) {
            CHECK_FALSE(g[v].empty());
        }
    }

    TEST_CASE("Graph with int properties") {
        Graph<int> g;

        auto v0 = g.add_vertex(100);
        auto v1 = g.add_vertex(200);
        auto v2 = g.add_vertex(300);
        auto v3 = g.add_vertex(400);

        // Two SCCs
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        g.add_edge(v2, v3, 1.0, EdgeType::Directed);
        g.add_edge(v3, v2, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(g);

        CHECK(result.num_components == 2);
        for (const auto &comp : result.components) {
            CHECK(comp.size() == 2);
        }
    }
}
