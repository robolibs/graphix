#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "graphix/vertex/algorithms/astar.hpp"
#include "graphix/vertex/graph.hpp"
#include <cmath>

using namespace graphix::vertex;
using namespace graphix::vertex::algorithms;

// Zero heuristic (equivalent to Dijkstra)
auto zero_heuristic = [](size_t, size_t) { return 0.0; };

TEST_SUITE("A* - Basic Functionality") {
    TEST_CASE("Empty graph") {
        Graph<void> g;
        auto result = astar(g, 0, 1, zero_heuristic);
        CHECK_FALSE(result.found);
        CHECK(std::isinf(result.distance));
        CHECK(result.path.empty());
        CHECK(result.nodes_explored == 0);
    }

    TEST_CASE("Single vertex - source equals target") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto result = astar(g, v0, v0, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 0.0);
        CHECK(result.path.size() == 1);
        CHECK(result.path[0] == v0);
        CHECK(result.nodes_explored == 1);
    }

    TEST_CASE("Two vertices, one edge") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 5.0);

        auto result = astar(g, v0, v1, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 5.0);
        REQUIRE(result.path.size() == 2);
        CHECK(result.path[0] == v0);
        CHECK(result.path[1] == v1);
        CHECK(result.nodes_explored >= 2);
    }

    TEST_CASE("Linear chain") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 2.0);
        g.add_edge(v2, v3, 3.0);

        auto result = astar(g, v0, v3, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 6.0);
        REQUIRE(result.path.size() == 4);
        CHECK(result.path[0] == v0);
        CHECK(result.path[1] == v1);
        CHECK(result.path[2] == v2);
        CHECK(result.path[3] == v3);
    }

    TEST_CASE("Unreachable target") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0);
        // v2 is disconnected

        auto result = astar(g, v0, v2, zero_heuristic);

        CHECK_FALSE(result.found);
        CHECK(std::isinf(result.distance));
        CHECK(result.path.empty());
    }
}

TEST_SUITE("A* - Heuristic-guided Search") {
    TEST_CASE("Simple grid 3x3 - Manhattan distance") {
        // Grid layout (indices):
        // 0 1 2
        // 3 4 5
        // 6 7 8
        Graph<void> g;
        for (int i = 0; i < 9; i++) {
            g.add_vertex();
        }

        // Horizontal edges
        g.add_edge(0, 1, 1.0);
        g.add_edge(1, 2, 1.0);
        g.add_edge(3, 4, 1.0);
        g.add_edge(4, 5, 1.0);
        g.add_edge(6, 7, 1.0);
        g.add_edge(7, 8, 1.0);

        // Vertical edges
        g.add_edge(0, 3, 1.0);
        g.add_edge(3, 6, 1.0);
        g.add_edge(1, 4, 1.0);
        g.add_edge(4, 7, 1.0);
        g.add_edge(2, 5, 1.0);
        g.add_edge(5, 8, 1.0);

        // Manhattan heuristic for 3x3 grid
        auto manhattan_3x3 = [](size_t v, size_t target) { return manhattan_distance(v, target, 3); };

        // Path from top-left (0) to bottom-right (8)
        auto result = astar(g, 0, 8, manhattan_3x3);

        CHECK(result.found);
        CHECK(result.distance == 4.0);     // Shortest path length
        CHECK(result.path.size() == 5);    // 5 nodes in path
        CHECK(result.nodes_explored <= 9); // Should explore fewer nodes than Dijkstra
    }

    TEST_CASE("Grid with perfect heuristic explores minimal nodes") {
        Graph<void> g;
        for (int i = 0; i < 9; i++) {
            g.add_vertex();
        }

        // Create grid edges
        g.add_edge(0, 1, 1.0);
        g.add_edge(1, 2, 1.0);
        g.add_edge(3, 4, 1.0);
        g.add_edge(4, 5, 1.0);
        g.add_edge(6, 7, 1.0);
        g.add_edge(7, 8, 1.0);
        g.add_edge(0, 3, 1.0);
        g.add_edge(3, 6, 1.0);
        g.add_edge(1, 4, 1.0);
        g.add_edge(4, 7, 1.0);
        g.add_edge(2, 5, 1.0);
        g.add_edge(5, 8, 1.0);

        auto manhattan_3x3 = [](size_t v, size_t target) { return manhattan_distance(v, target, 3); };

        // Compare with zero heuristic
        auto result_astar = astar(g, 0, 8, manhattan_3x3);
        auto result_dijkstra = astar(g, 0, 8, zero_heuristic);

        CHECK(result_astar.found);
        CHECK(result_dijkstra.found);
        CHECK(result_astar.distance == result_dijkstra.distance);             // Same distance
        CHECK(result_astar.nodes_explored <= result_dijkstra.nodes_explored); // A* explores fewer
    }

    TEST_CASE("Consistent heuristic gives optimal path") {
        // Triangle graph
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 10.0);
        g.add_edge(v0, v2, 5.0);
        g.add_edge(v2, v1, 3.0);

        // Admissible heuristic (never overestimates)
        auto h = [&](size_t v, size_t target) {
            if (v == v0 && target == v1)
                return 7.0; // Less than actual 8.0
            if (v == v2 && target == v1)
                return 2.0; // Less than actual 3.0
            return 0.0;
        };

        auto result = astar(g, v0, v1, h);

        CHECK(result.found);
        CHECK(result.distance == 8.0); // Via v2
        REQUIRE(result.path.size() == 3);
        CHECK(result.path[0] == v0);
        CHECK(result.path[1] == v2);
        CHECK(result.path[2] == v1);
    }
}

TEST_SUITE("A* - Multiple Paths") {
    TEST_CASE("Diamond graph - chooses shortest") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Two paths from v0 to v3
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v3, 1.0); // Path 1: cost 2
        g.add_edge(v0, v2, 5.0);
        g.add_edge(v2, v3, 5.0); // Path 2: cost 10

        auto result = astar(g, v0, v3, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 2.0);
        REQUIRE(result.path.size() == 3);
        CHECK(result.path[1] == v1); // Goes via v1, not v2
    }

    TEST_CASE("Multiple equal-cost paths") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Two equal-cost paths
        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v3, 1.0); // Path 1: cost 2
        g.add_edge(v0, v2, 1.0);
        g.add_edge(v2, v3, 1.0); // Path 2: cost 2

        auto result = astar(g, v0, v3, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 2.0);
        CHECK(result.path.size() == 3);
        // Either path is valid
    }
}

TEST_SUITE("A* - Edge Cases") {
    TEST_CASE("Self-loop ignored") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();

        g.add_edge(v0, v0, 10.0); // Self-loop
        g.add_edge(v0, v1, 5.0);

        auto result = astar(g, v0, v1, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 5.0);
    }

    TEST_CASE("Zero-weight edges") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 0.0);
        g.add_edge(v1, v2, 0.0);

        auto result = astar(g, v0, v2, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 0.0);
        CHECK(result.path.size() == 3);
    }

    TEST_CASE("Large weights") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();

        g.add_edge(v0, v1, 1000000.0);

        auto result = astar(g, v0, v1, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 1000000.0);
    }
}

TEST_SUITE("A* - Directed Graphs") {
    TEST_CASE("Simple directed path") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        auto result = astar(g, v0, v2, zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 2.0);
    }

    TEST_CASE("Directed - no backward path") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);

        auto result = astar(g, v1, v0, zero_heuristic); // Reverse direction

        CHECK_FALSE(result.found); // No path backwards
    }

    TEST_CASE("Mixed directed/undirected") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);

        auto result1 = astar(g, v0, v2, zero_heuristic);
        CHECK(result1.found);
        CHECK(result1.distance == 2.0);

        auto result2 = astar(g, v2, v0, zero_heuristic);
        CHECK_FALSE(result2.found); // Can't go back through directed edge
    }
}

TEST_SUITE("A* - Vertex Properties") {
    struct Point {
        double x, y;
    };

    TEST_CASE("Graph with vertex properties") {
        Graph<Point> g;
        auto v0 = g.add_vertex({0.0, 0.0});
        auto v1 = g.add_vertex({1.0, 0.0});
        auto v2 = g.add_vertex({2.0, 0.0});

        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 1.0);

        // Euclidean heuristic based on vertex properties
        auto euclidean_h = [&g](size_t v, size_t target) { return euclidean_distance(g[v], g[target]); };

        auto result = astar(g, v0, v2, euclidean_h);

        CHECK(result.found);
        CHECK(result.distance == 2.0);
        CHECK(result.path.size() == 3);
    }
}

TEST_SUITE("A* - Performance Comparison") {
    TEST_CASE("A* with heuristic vs Dijkstra (zero heuristic)") {
        // Larger grid 5x5
        Graph<void> g;
        for (int i = 0; i < 25; i++) {
            g.add_vertex();
        }

        // Create grid edges (5x5)
        for (int row = 0; row < 5; row++) {
            for (int col = 0; col < 5; col++) {
                int idx = row * 5 + col;
                if (col < 4)
                    g.add_edge(idx, idx + 1, 1.0); // Right
                if (row < 4)
                    g.add_edge(idx, idx + 5, 1.0); // Down
            }
        }

        auto manhattan_5x5 = [](size_t v, size_t target) { return manhattan_distance(v, target, 5); };

        // Path from (0,0) to (4,4)
        auto result_astar = astar(g, 0, 24, manhattan_5x5);
        auto result_dijkstra = astar(g, 0, 24, zero_heuristic);

        // Both find same optimal path
        CHECK(result_astar.found);
        CHECK(result_dijkstra.found);
        CHECK(result_astar.distance == result_dijkstra.distance);
        CHECK(result_astar.distance == 8.0); // 4 right + 4 down

        // A* explores fewer nodes
        CHECK(result_astar.nodes_explored < result_dijkstra.nodes_explored);
    }
}

TEST_SUITE("A* - Helper Functions") {
    TEST_CASE("astar_dijkstra convenience function") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0);
        g.add_edge(v1, v2, 2.0);

        auto result = astar_dijkstra(g, v0, v2);

        CHECK(result.found);
        CHECK(result.distance == 3.0);
        CHECK(result.path.size() == 3);
    }

    TEST_CASE("Manhattan distance helper - horizontal") {
        double dist = manhattan_distance(0, 2, 5); // (0,0) to (0,2) in 5x5 grid
        CHECK(dist == 2.0);
    }

    TEST_CASE("Manhattan distance helper - vertical") {
        double dist = manhattan_distance(0, 10, 5); // (0,0) to (2,0) in 5x5 grid
        CHECK(dist == 2.0);
    }

    TEST_CASE("Manhattan distance helper - diagonal") {
        double dist = manhattan_distance(0, 12, 5); // (0,0) to (2,2) in 5x5 grid
        CHECK(dist == 4.0);
    }

    TEST_CASE("Euclidean distance helper") {
        struct Point {
            double x, y;
        };
        Point p1 = {0.0, 0.0};
        Point p2 = {3.0, 4.0};

        double dist = euclidean_distance(p1, p2);
        CHECK(dist == doctest::Approx(5.0));
    }
}

TEST_SUITE("A* - Complex Scenarios") {
    TEST_CASE("Maze-like graph") {
        // Create a small maze
        Graph<void> g;
        for (int i = 0; i < 12; i++) {
            g.add_vertex();
        }

        // Maze structure (not all connections)
        g.add_edge(0, 1, 1.0);
        g.add_edge(1, 2, 1.0);
        g.add_edge(2, 5, 1.0);
        g.add_edge(0, 3, 1.0);
        g.add_edge(3, 6, 1.0);
        g.add_edge(6, 7, 1.0);
        g.add_edge(7, 8, 1.0);
        g.add_edge(8, 11, 1.0);

        auto manhattan_4x3 = [](size_t v, size_t target) { return manhattan_distance(v, target, 3); };

        auto result = astar(g, 0, 11, manhattan_4x3);

        CHECK(result.found);
        CHECK(result.path.size() > 0);
    }

    TEST_CASE("Complete graph - all vertices connected") {
        Graph<void> g;
        std::vector<size_t> vertices;
        for (int i = 0; i < 5; i++) {
            vertices.push_back(g.add_vertex());
        }

        // Connect all pairs
        for (size_t i = 0; i < vertices.size(); i++) {
            for (size_t j = i + 1; j < vertices.size(); j++) {
                g.add_edge(vertices[i], vertices[j], 1.0);
            }
        }

        auto result = astar(g, vertices[0], vertices[4], zero_heuristic);

        CHECK(result.found);
        CHECK(result.distance == 1.0); // Direct edge
        CHECK(result.path.size() == 2);
    }
}
