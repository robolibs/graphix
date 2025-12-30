#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "graphix/vertex/algorithms/bellman_ford.hpp"
#include "graphix/vertex/graph.hpp"
#include <cmath>

using namespace graphix::vertex;

TEST_SUITE("Bellman-Ford - Basic Functionality") {
    TEST_CASE("Empty graph") {
        Graph<void> g;
        auto result = bellman_ford(g, 0);
        CHECK(result.distances.empty());
        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Single vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v0] == 0.0);
        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Two vertices, one edge") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 5.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v0] == 0.0);
        CHECK(result.distances[v1] == 5.0);
        CHECK(result.predecessors[v1] == v0);
        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Linear chain") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 2.0, EdgeType::Directed);
        g.add_edge(v2, v3, 3.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v0] == 0.0);
        CHECK(result.distances[v1] == 1.0);
        CHECK(result.distances[v2] == 3.0);
        CHECK(result.distances[v3] == 6.0);
        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Unreachable vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        // v2 is unreachable from v0

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v0] == 0.0);
        CHECK(result.distances[v1] == 1.0);
        CHECK(std::isinf(result.distances[v2]));
    }
}

TEST_SUITE("Bellman-Ford - Negative Weights") {
    TEST_CASE("Single negative edge") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, -5.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v0] == 0.0);
        CHECK(result.distances[v1] == -5.0);
        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Path with negative weights (no cycle)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 4.0, EdgeType::Directed);
        g.add_edge(v1, v2, -3.0, EdgeType::Directed);
        g.add_edge(v2, v3, 2.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v0] == 0.0);
        CHECK(result.distances[v1] == 4.0);
        CHECK(result.distances[v2] == 1.0);
        CHECK(result.distances[v3] == 3.0);
        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Better path via negative edge") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v2, 10.0, EdgeType::Directed); // Direct: 10
        g.add_edge(v0, v1, 3.0, EdgeType::Directed);
        g.add_edge(v1, v2, -5.0, EdgeType::Directed); // Via v1: 3 + (-5) = -2

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v2] == -2.0); // Should take negative path
        CHECK(result.predecessors[v2] == v1);
    }
}

TEST_SUITE("Bellman-Ford - Negative Cycles") {
    TEST_CASE("Simple negative cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, -3.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed); // Cycle total: 1 + (-3) + 1 = -1

        auto result = bellman_ford(g, v0);

        CHECK(result.has_negative_cycle);
        CHECK_FALSE(result.negative_cycle.empty());
    }

    TEST_CASE("Negative cycle not reachable from source") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Disconnected negative cycle
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, -3.0, EdgeType::Directed);
        g.add_edge(v3, v1, 1.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        // Since cycle is not reachable from v0, it won't be detected
        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Zero-weight cycle (not negative)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, -1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 0.0, EdgeType::Directed); // Total: 1 + (-1) + 0 = 0

        auto result = bellman_ford(g, v0);

        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Positive cycle (not negative)") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 2.0, EdgeType::Directed);
        g.add_edge(v1, v2, 3.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed); // Total: 2 + 3 + 1 = 6

        auto result = bellman_ford(g, v0);

        CHECK_FALSE(result.has_negative_cycle);
    }
}

TEST_SUITE("Bellman-Ford - Multiple Paths") {
    TEST_CASE("Diamond with different path costs") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 4.0, EdgeType::Directed);
        g.add_edge(v1, v3, 3.0, EdgeType::Directed); // Path via v1: 1 + 3 = 4
        g.add_edge(v2, v3, 1.0, EdgeType::Directed); // Path via v2: 4 + 1 = 5

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v3] == 4.0); // Shorter path via v1
        CHECK(result.predecessors[v3] == v1);
    }

    TEST_CASE("Multiple paths with negative weights") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 5.0, EdgeType::Directed);
        g.add_edge(v0, v2, 2.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);  // Path 1: 5 + 1 = 6
        g.add_edge(v2, v3, -1.0, EdgeType::Directed); // Path 2: 2 + (-1) = 1

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v3] == 1.0);
        CHECK(result.predecessors[v3] == v2);
    }
}

TEST_SUITE("Bellman-Ford - Helper Functions") {
    TEST_CASE("get_shortest_path - simple path") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 2.0, EdgeType::Directed);

        auto path = get_shortest_path(g, v0, v2);

        REQUIRE(path.size() == 3);
        CHECK(path[0] == v0);
        CHECK(path[1] == v1);
        CHECK(path[2] == v2);
    }

    TEST_CASE("get_shortest_path - no path") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        // No path from v0 to v2

        auto path = get_shortest_path(g, v0, v2);

        CHECK(path.empty());
    }

    TEST_CASE("get_shortest_path - with negative cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, -3.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto path = get_shortest_path(g, v0, v2);

        CHECK(path.empty()); // No valid path with negative cycle
    }

    TEST_CASE("has_negative_cycle - true") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, -3.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        CHECK(has_negative_cycle(g, v0));
    }

    TEST_CASE("has_negative_cycle - false") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 2.0, EdgeType::Directed);

        CHECK_FALSE(has_negative_cycle(g, v0));
    }

    TEST_CASE("get_distance") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 3.0, EdgeType::Directed);
        g.add_edge(v1, v2, -1.0, EdgeType::Directed);

        CHECK(get_distance(g, v0, v0) == 0.0);
        CHECK(get_distance(g, v0, v1) == 3.0);
        CHECK(get_distance(g, v0, v2) == 2.0);
    }
}

TEST_SUITE("Bellman-Ford - Edge Cases") {
    TEST_CASE("Self-loop with negative weight") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        g.add_edge(v0, v0, -1.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK(result.has_negative_cycle);
    }

    TEST_CASE("Self-loop with positive weight") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        g.add_edge(v0, v0, 5.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK_FALSE(result.has_negative_cycle);
        CHECK(result.distances[v0] == 0.0); // Self-loop ignored for shortest path
    }

    TEST_CASE("Undirected edges are ignored") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 2.0, EdgeType::Undirected); // Ignored

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v1] == 1.0);
        CHECK(std::isinf(result.distances[v2])); // Unreachable via directed edges
    }

    TEST_CASE("Large graph with negative weights") {
        Graph<void> g;
        std::vector<size_t> vertices;

        for (int i = 0; i < 10; i++) {
            vertices.push_back(g.add_vertex());
        }

        // Create chain with alternating positive/negative weights
        for (int i = 0; i < 9; i++) {
            double weight = (i % 2 == 0) ? 5.0 : -2.0;
            g.add_edge(vertices[i], vertices[i + 1], weight, EdgeType::Directed);
        }

        auto result = bellman_ford(g, vertices[0]);

        CHECK_FALSE(result.has_negative_cycle);
        // Distance to last vertex should be sum of alternating weights
        // 5 + (-2) + 5 + (-2) + 5 + (-2) + 5 + (-2) + 5 = 17
        CHECK(result.distances[vertices[9]] == 17.0);
    }

    TEST_CASE("Disconnected components") {
        Graph<void> g;

        // Component 1
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 2.0, EdgeType::Directed);

        // Component 2 (separate)
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        g.add_edge(v2, v3, 3.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v1] == 2.0);
        CHECK(std::isinf(result.distances[v2]));
        CHECK(std::isinf(result.distances[v3]));
    }
}

TEST_SUITE("Bellman-Ford - With Vertex Properties") {
    TEST_CASE("Graph with string properties") {
        Graph<std::string> g;

        auto v0 = g.add_vertex("Start");
        auto v1 = g.add_vertex("Middle");
        auto v2 = g.add_vertex("End");

        g.add_edge(v0, v1, 2.0, EdgeType::Directed);
        g.add_edge(v1, v2, -1.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK(result.distances[v0] == 0.0);
        CHECK(result.distances[v1] == 2.0);
        CHECK(result.distances[v2] == 1.0);
        CHECK_FALSE(result.has_negative_cycle);
    }

    TEST_CASE("Graph with int properties and negative cycle") {
        Graph<int> g;

        auto v0 = g.add_vertex(100);
        auto v1 = g.add_vertex(200);
        auto v2 = g.add_vertex(300);

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, -3.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto result = bellman_ford(g, v0);

        CHECK(result.has_negative_cycle);
    }
}
