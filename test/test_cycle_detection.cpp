#include "graphix/vertex/algorithms/cycle_detection.hpp"
#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

using namespace graphix::vertex;

TEST_SUITE("Cycle Detection - Directed Graphs") {
    TEST_CASE("Empty directed graph") {
        Graph<void> g;
        auto result = find_cycle_directed(g);
        CHECK_FALSE(result.has_cycle);
        CHECK(result.cycle.empty());
        CHECK_FALSE(has_cycle_directed(g));
    }

    TEST_CASE("Single vertex, no edges") {
        Graph<void> g;
        g.add_vertex();
        auto result = find_cycle_directed(g);
        CHECK_FALSE(result.has_cycle);
        CHECK_FALSE(has_cycle_directed(g));
    }

    TEST_CASE("Two vertices, one directed edge (no cycle)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK_FALSE(result.has_cycle);
        CHECK_FALSE(has_cycle_directed(g));
    }

    TEST_CASE("Simple cycle: 0 -> 1 -> 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK(result.has_cycle);
        CHECK(has_cycle_directed(g));
        REQUIRE(result.cycle.size() >= 2);

        // Verify it's actually a cycle
        CHECK(std::find(result.cycle.begin(), result.cycle.end(), v0) != result.cycle.end());
        CHECK(std::find(result.cycle.begin(), result.cycle.end(), v1) != result.cycle.end());
    }

    TEST_CASE("Self-loop: 0 -> 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        g.add_edge(v0, v0, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK(result.has_cycle);
        CHECK(has_cycle_directed(g));
        REQUIRE_FALSE(result.cycle.empty());
    }

    TEST_CASE("Triangle cycle: 0 -> 1 -> 2 -> 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK(result.has_cycle);
        REQUIRE(result.cycle.size() >= 3);
    }

    TEST_CASE("DAG: no cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Diamond DAG
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK_FALSE(result.has_cycle);
        CHECK(result.cycle.empty());
    }

    TEST_CASE("Cycle with tail: 0 -> 1 -> 2 -> 1") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v1, 1.0, EdgeType::Directed); // Cycle between 1 and 2

        auto result = find_cycle_directed(g);
        CHECK(result.has_cycle);
        REQUIRE(result.cycle.size() >= 2);

        // The cycle should involve v1 and v2
        CHECK(std::find(result.cycle.begin(), result.cycle.end(), v1) != result.cycle.end());
        CHECK(std::find(result.cycle.begin(), result.cycle.end(), v2) != result.cycle.end());
    }

    TEST_CASE("Multiple disconnected cycles") {
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

        auto result = find_cycle_directed(g);
        CHECK(result.has_cycle);
        // Should find at least one cycle
        REQUIRE_FALSE(result.cycle.empty());
    }

    TEST_CASE("Large cycle") {
        Graph<void> g;
        std::vector<size_t> vertices;
        for (int i = 0; i < 10; i++) {
            vertices.push_back(g.add_vertex());
        }

        // Create cycle: 0 -> 1 -> 2 -> ... -> 9 -> 0
        for (int i = 0; i < 10; i++) {
            g.add_edge(vertices[i], vertices[(i + 1) % 10], 1.0, EdgeType::Directed);
        }

        auto result = find_cycle_directed(g);
        CHECK(result.has_cycle);
        REQUIRE(result.cycle.size() >= 2);
    }

    TEST_CASE("Linear chain (no cycle)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK_FALSE(result.has_cycle);
    }
}

TEST_SUITE("Cycle Detection - Undirected Graphs") {
    TEST_CASE("Empty undirected graph") {
        Graph<void> g;
        auto result = find_cycle_undirected(g);
        CHECK_FALSE(result.has_cycle);
        CHECK(result.cycle.empty());
        CHECK_FALSE(has_cycle_undirected(g));
    }

    TEST_CASE("Single vertex, no edges") {
        Graph<void> g;
        g.add_vertex();
        auto result = find_cycle_undirected(g);
        CHECK_FALSE(result.has_cycle);
    }

    TEST_CASE("Two vertices, one edge (no cycle)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK_FALSE(result.has_cycle);
        CHECK_FALSE(has_cycle_undirected(g));
    }

    TEST_CASE("Tree structure (no cycle)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Tree: 0 connected to 1, 2, 3
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v0, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v0, v3, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK_FALSE(result.has_cycle);
    }

    TEST_CASE("Triangle cycle: 0 - 1 - 2 - 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v0, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK(result.has_cycle);
        CHECK(has_cycle_undirected(g));
        REQUIRE(result.cycle.size() >= 3);
    }

    TEST_CASE("Square cycle: 0 - 1 - 2 - 3 - 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v3, 1.0, EdgeType::Undirected);
        g.add_edge(v3, v0, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK(result.has_cycle);
        REQUIRE(result.cycle.size() >= 4);
    }

    TEST_CASE("Linear path (no cycle)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v3, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK_FALSE(result.has_cycle);
    }

    TEST_CASE("Cycle with branch") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        auto v4 = g.add_vertex();

        // Triangle cycle: 0 - 1 - 2 - 0
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v0, 1.0, EdgeType::Undirected);

        // Branch from v0
        g.add_edge(v0, v3, 1.0, EdgeType::Undirected);
        g.add_edge(v3, v4, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK(result.has_cycle);
    }

    TEST_CASE("Multiple disconnected cycles") {
        Graph<void> g;

        // Cycle 1: 0 - 1 - 2 - 0
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v0, 1.0, EdgeType::Undirected);

        // Cycle 2: 3 - 4 - 5 - 3
        auto v3 = g.add_vertex();
        auto v4 = g.add_vertex();
        auto v5 = g.add_vertex();
        g.add_edge(v3, v4, 1.0, EdgeType::Undirected);
        g.add_edge(v4, v5, 1.0, EdgeType::Undirected);
        g.add_edge(v5, v3, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK(result.has_cycle);
    }

    TEST_CASE("Large cycle") {
        Graph<void> g;
        std::vector<size_t> vertices;
        for (int i = 0; i < 20; i++) {
            vertices.push_back(g.add_vertex());
        }

        // Create cycle
        for (int i = 0; i < 20; i++) {
            g.add_edge(vertices[i], vertices[(i + 1) % 20], 1.0, EdgeType::Undirected);
        }

        auto result = find_cycle_undirected(g);
        CHECK(result.has_cycle);
    }
}

TEST_SUITE("Cycle Detection - General (Auto-detect)") {
    TEST_CASE("Auto-detect directed cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto result = find_cycle(g);
        CHECK(result.has_cycle);
        CHECK(has_cycle(g));
    }

    TEST_CASE("Auto-detect undirected cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v0, 1.0, EdgeType::Undirected);

        auto result = find_cycle(g);
        CHECK(result.has_cycle);
        CHECK(has_cycle(g));
    }

    TEST_CASE("Auto-detect no cycle in directed DAG") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        auto result = find_cycle(g);
        CHECK_FALSE(result.has_cycle);
        CHECK_FALSE(has_cycle(g));
    }

    TEST_CASE("Auto-detect no cycle in undirected tree") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);

        auto result = find_cycle(g);
        CHECK_FALSE(result.has_cycle);
        CHECK_FALSE(has_cycle(g));
    }

    TEST_CASE("Mixed graph with directed cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Directed cycle
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        // Undirected edges (won't form cycle for directed check)
        g.add_edge(v2, v3, 1.0, EdgeType::Undirected);

        auto result = find_cycle(g);
        CHECK(result.has_cycle);
    }

    TEST_CASE("Empty graph") {
        Graph<void> g;
        auto result = find_cycle(g);
        CHECK_FALSE(result.has_cycle);
        CHECK_FALSE(has_cycle(g));
    }
}

TEST_SUITE("Cycle Detection - With Vertex Properties") {
    TEST_CASE("Directed cycle with string properties") {
        Graph<std::string> g;
        auto v0 = g.add_vertex("Node A");
        auto v1 = g.add_vertex("Node B");
        auto v2 = g.add_vertex("Node C");

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK(result.has_cycle);
        REQUIRE(result.cycle.size() >= 3);

        // Verify properties are preserved
        for (auto v : result.cycle) {
            CHECK_FALSE(g[v].empty());
        }
    }

    TEST_CASE("Undirected cycle with int properties") {
        Graph<int> g;
        auto v0 = g.add_vertex(100);
        auto v1 = g.add_vertex(200);
        auto v2 = g.add_vertex(300);

        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v0, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK(result.has_cycle);
        REQUIRE(result.cycle.size() >= 3);
    }

    TEST_CASE("No cycle with properties") {
        Graph<std::string> g;
        auto v0 = g.add_vertex("Start");
        auto v1 = g.add_vertex("Middle");
        auto v2 = g.add_vertex("End");

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        auto result = find_cycle(g);
        CHECK_FALSE(result.has_cycle);
    }
}

TEST_SUITE("Cycle Detection - Edge Cases") {
    TEST_CASE("Isolated vertices (no edges)") {
        Graph<void> g;
        g.add_vertex();
        g.add_vertex();
        g.add_vertex();

        auto result = find_cycle(g);
        CHECK_FALSE(result.has_cycle);
    }

    TEST_CASE("Complete directed graph (many cycles)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // Complete graph: all vertices connected to all others
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);
        g.add_edge(v2, v1, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK(result.has_cycle);
    }

    TEST_CASE("Complete undirected graph (has cycle)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v0, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);

        auto result = find_cycle_undirected(g);
        CHECK(result.has_cycle);
    }

    TEST_CASE("Directed graph with multiple paths but no cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Multiple paths from v0 to v3, but no cycle
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = find_cycle_directed(g);
        CHECK_FALSE(result.has_cycle);
    }

    TEST_CASE("Star graph (undirected, no cycle)") {
        Graph<void> g;
        auto center = g.add_vertex();
        std::vector<size_t> leaves;

        for (int i = 0; i < 5; i++) {
            auto leaf = g.add_vertex();
            g.add_edge(center, leaf, 1.0, EdgeType::Undirected);
            leaves.push_back(leaf);
        }

        auto result = find_cycle_undirected(g);
        CHECK_FALSE(result.has_cycle);
    }
}
