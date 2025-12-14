#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "graphix/vertex/algorithms/topological_sort.hpp"
#include "graphix/vertex/graph.hpp"

using namespace graphix::vertex;

TEST_SUITE("Topological Sort") {
    TEST_CASE("Empty graph") {
        Graph<void> g;
        auto result = topological_sort(g);
        CHECK(result.is_dag);
        CHECK(result.order.empty());
        CHECK(result.cycle.empty());
    }

    TEST_CASE("Single vertex") {
        Graph<void> g;
        auto v0 = g.add_vertex();

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        CHECK(result.order.size() == 1);
        CHECK(result.order[0] == v0);
    }

    TEST_CASE("Two vertices, one directed edge") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 2);

        // v0 must come before v1
        auto it0 = std::find(result.order.begin(), result.order.end(), v0);
        auto it1 = std::find(result.order.begin(), result.order.end(), v1);
        CHECK(it0 < it1);
    }

    TEST_CASE("Linear chain: 0 -> 1 -> 2 -> 3") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 4);

        // Verify ordering constraints
        auto it0 = std::find(result.order.begin(), result.order.end(), v0);
        auto it1 = std::find(result.order.begin(), result.order.end(), v1);
        auto it2 = std::find(result.order.begin(), result.order.end(), v2);
        auto it3 = std::find(result.order.begin(), result.order.end(), v3);

        CHECK(it0 < it1);
        CHECK(it1 < it2);
        CHECK(it2 < it3);
    }

    TEST_CASE("Diamond DAG: 0 -> {1, 2} -> 3") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 4);

        // v0 must come before v1, v2, v3
        auto it0 = std::find(result.order.begin(), result.order.end(), v0);
        auto it1 = std::find(result.order.begin(), result.order.end(), v1);
        auto it2 = std::find(result.order.begin(), result.order.end(), v2);
        auto it3 = std::find(result.order.begin(), result.order.end(), v3);

        CHECK(it0 < it1);
        CHECK(it0 < it2);
        CHECK(it0 < it3);
        CHECK(it1 < it3);
        CHECK(it2 < it3);
    }

    TEST_CASE("Multiple source nodes") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Two independent chains: 0->2 and 1->3
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 4);

        // Only constraint: sources before their targets
        auto it0 = std::find(result.order.begin(), result.order.end(), v0);
        auto it1 = std::find(result.order.begin(), result.order.end(), v1);
        auto it2 = std::find(result.order.begin(), result.order.end(), v2);
        auto it3 = std::find(result.order.begin(), result.order.end(), v3);

        CHECK(it0 < it2);
        CHECK(it1 < it3);
    }

    TEST_CASE("Isolated vertices") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // No edges - all isolated
        auto result = topological_sort(g);
        CHECK(result.is_dag);
        CHECK(result.order.size() == 3);

        // All vertices should be present
        CHECK(std::find(result.order.begin(), result.order.end(), v0) != result.order.end());
        CHECK(std::find(result.order.begin(), result.order.end(), v1) != result.order.end());
        CHECK(std::find(result.order.begin(), result.order.end(), v2) != result.order.end());
    }

    TEST_CASE("Simple cycle: 0 -> 1 -> 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v0, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK_FALSE(result.is_dag);
        CHECK(result.order.empty());
    }

    TEST_CASE("Self-loop: 0 -> 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        g.add_edge(v0, v0, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK_FALSE(result.is_dag);
        CHECK(result.order.empty());
    }

    TEST_CASE("Cycle in larger graph: 0 -> 1 -> 2 -> 0") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK_FALSE(result.is_dag);
        CHECK(result.order.empty());
    }

    TEST_CASE("Cycle with tail: 0 -> 1 -> 2 -> 1, 2 -> 3") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v1, 1.0, EdgeType::Directed); // Cycle
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK_FALSE(result.is_dag);
    }

    TEST_CASE("Undirected edges are ignored") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        // Mix of directed and undirected
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected); // Should be ignored

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 3);

        // Only constraint: v0 before v1
        auto it0 = std::find(result.order.begin(), result.order.end(), v0);
        auto it1 = std::find(result.order.begin(), result.order.end(), v1);
        CHECK(it0 < it1);
    }

    TEST_CASE("Complex DAG - build system dependencies") {
        Graph<std::string> g;

        // Files: main.cpp -> main.o -> executable
        //        util.cpp -> util.o -> executable
        //        util.h -> {main.o, util.o}

        auto main_cpp = g.add_vertex("main.cpp");
        auto util_cpp = g.add_vertex("util.cpp");
        auto util_h = g.add_vertex("util.h");
        auto main_o = g.add_vertex("main.o");
        auto util_o = g.add_vertex("util.o");
        auto exe = g.add_vertex("executable");

        g.add_edge(main_cpp, main_o, 1.0, EdgeType::Directed);
        g.add_edge(util_h, main_o, 1.0, EdgeType::Directed);
        g.add_edge(util_cpp, util_o, 1.0, EdgeType::Directed);
        g.add_edge(util_h, util_o, 1.0, EdgeType::Directed);
        g.add_edge(main_o, exe, 1.0, EdgeType::Directed);
        g.add_edge(util_o, exe, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 6);

        // Verify dependencies
        auto it_main_cpp = std::find(result.order.begin(), result.order.end(), main_cpp);
        auto it_util_cpp = std::find(result.order.begin(), result.order.end(), util_cpp);
        auto it_util_h = std::find(result.order.begin(), result.order.end(), util_h);
        auto it_main_o = std::find(result.order.begin(), result.order.end(), main_o);
        auto it_util_o = std::find(result.order.begin(), result.order.end(), util_o);
        auto it_exe = std::find(result.order.begin(), result.order.end(), exe);

        CHECK(it_main_cpp < it_main_o);
        CHECK(it_util_h < it_main_o);
        CHECK(it_util_cpp < it_util_o);
        CHECK(it_util_h < it_util_o);
        CHECK(it_main_o < it_exe);
        CHECK(it_util_o < it_exe);
    }

    TEST_CASE("DFS-based topological sort") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);

        auto result = topological_sort_dfs(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 4);

        // Verify ordering constraints (same as Kahn's)
        auto it0 = std::find(result.order.begin(), result.order.end(), v0);
        auto it1 = std::find(result.order.begin(), result.order.end(), v1);
        auto it2 = std::find(result.order.begin(), result.order.end(), v2);
        auto it3 = std::find(result.order.begin(), result.order.end(), v3);

        CHECK(it0 < it1);
        CHECK(it0 < it2);
        CHECK(it0 < it3);
        CHECK(it1 < it3);
        CHECK(it2 < it3);
    }

    TEST_CASE("DFS detects cycle") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        auto result = topological_sort_dfs(g);
        CHECK_FALSE(result.is_dag);
    }

    TEST_CASE("Large DAG") {
        Graph<void> g;
        std::vector<size_t> vertices;

        // Create 100 vertices
        for (int i = 0; i < 100; i++) {
            vertices.push_back(g.add_vertex());
        }

        // Create a layered DAG: each vertex in layer i connects to vertices in layer i+1
        for (int i = 0; i < 90; i++) {
            g.add_edge(vertices[i], vertices[i + 10], 1.0, EdgeType::Directed);
        }

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        CHECK(result.order.size() == 100);

        // Verify all edge constraints are satisfied
        auto edges = g.edges();
        for (const auto &edge : edges) {
            if (edge.type == EdgeType::Directed) {
                auto it_src = std::find(result.order.begin(), result.order.end(), edge.source);
                auto it_tgt = std::find(result.order.begin(), result.order.end(), edge.target);
                CHECK(it_src < it_tgt);
            }
        }
    }

    TEST_CASE("Disconnected components") {
        Graph<void> g;

        // Component 1: 0 -> 1 -> 2
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        // Component 2: 3 -> 4
        auto v3 = g.add_vertex();
        auto v4 = g.add_vertex();
        g.add_edge(v3, v4, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 5);

        // Verify constraints within each component
        auto it0 = std::find(result.order.begin(), result.order.end(), v0);
        auto it1 = std::find(result.order.begin(), result.order.end(), v1);
        auto it2 = std::find(result.order.begin(), result.order.end(), v2);
        auto it3 = std::find(result.order.begin(), result.order.end(), v3);
        auto it4 = std::find(result.order.begin(), result.order.end(), v4);

        CHECK(it0 < it1);
        CHECK(it1 < it2);
        CHECK(it3 < it4);
    }

    TEST_CASE("Graph with vertex properties") {
        Graph<int> g;

        auto v0 = g.add_vertex(100);
        auto v1 = g.add_vertex(200);
        auto v2 = g.add_vertex(300);

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        REQUIRE(result.order.size() == 3);

        // Verify properties are preserved
        CHECK(g[result.order[0]] == 100);
        CHECK(g[result.order[1]] == 200);
        CHECK(g[result.order[2]] == 300);
    }

    TEST_CASE("All vertices have outgoing edges") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        // Tree structure
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        CHECK(result.order.size() == 4);

        // v0 should be first (only source)
        CHECK(result.order[0] == v0);
    }

    TEST_CASE("Binary tree structure") {
        Graph<void> g;
        auto v0 = g.add_vertex(); // root
        auto v1 = g.add_vertex(); // left child
        auto v2 = g.add_vertex(); // right child
        auto v3 = g.add_vertex(); // left-left
        auto v4 = g.add_vertex(); // left-right
        auto v5 = g.add_vertex(); // right-left
        auto v6 = g.add_vertex(); // right-right

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v0, v2, 1.0, EdgeType::Directed);
        g.add_edge(v1, v3, 1.0, EdgeType::Directed);
        g.add_edge(v1, v4, 1.0, EdgeType::Directed);
        g.add_edge(v2, v5, 1.0, EdgeType::Directed);
        g.add_edge(v2, v6, 1.0, EdgeType::Directed);

        auto result = topological_sort(g);
        CHECK(result.is_dag);
        CHECK(result.order.size() == 7);

        // Root should be first
        CHECK(result.order[0] == v0);

        // All descendants should come after ancestors
        auto it0 = std::find(result.order.begin(), result.order.end(), v0);
        auto it1 = std::find(result.order.begin(), result.order.end(), v1);
        auto it2 = std::find(result.order.begin(), result.order.end(), v2);
        auto it3 = std::find(result.order.begin(), result.order.end(), v3);
        auto it4 = std::find(result.order.begin(), result.order.end(), v4);

        CHECK(it0 < it1);
        CHECK(it0 < it2);
        CHECK(it1 < it3);
        CHECK(it1 < it4);
    }
}
