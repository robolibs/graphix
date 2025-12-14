#include "doctest/doctest.h"
#include "graphix/vertex/algorithms/graph_views.hpp"
#include "graphix/vertex/graph.hpp"

using namespace graphix::vertex;
using namespace graphix::vertex::views;

TEST_SUITE("Graph Views - ReversedGraphView") {

    TEST_CASE("Reversed empty graph") {
        Graph<void> g;
        auto view = reversed(g);

        CHECK(view.vertex_count() == 0);
        CHECK(view.edge_count() == 0);
    }

    TEST_CASE("Reversed graph with only undirected edges") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 2.0, EdgeType::Undirected);

        auto view = reversed(g);

        CHECK(view.vertex_count() == 3);
        CHECK(view.edge_count() == 2);

        // Undirected edges should be unchanged
        auto edges = view.edges();
        CHECK(edges.size() == 2);
    }

    TEST_CASE("Reversed graph with directed edges") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed); // 0 -> 1
        g.add_edge(v1, v2, 2.0, EdgeType::Directed); // 1 -> 2

        auto view = reversed(g);

        CHECK(view.vertex_count() == 3);
        CHECK(view.edge_count() == 2);

        // Edges should be reversed
        auto edges = view.edges();
        bool found_1_to_0 = false;
        bool found_2_to_1 = false;
        for (const auto &e : edges) {
            if (e.source == 1 && e.target == 0)
                found_1_to_0 = true;
            if (e.source == 2 && e.target == 1)
                found_2_to_1 = true;
        }
        CHECK(found_1_to_0);
        CHECK(found_2_to_1);
    }

    TEST_CASE("Reversed graph neighbors") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed); // 0 -> 1
        g.add_edge(v0, v2, 1.0, EdgeType::Directed); // 0 -> 2

        auto view = reversed(g);

        // In original: v0 has neighbors v1, v2
        // In reversed: v1 and v2 have neighbor v0
        auto neighbors_v1 = view.neighbors(v1);
        CHECK(neighbors_v1.size() == 1);
        CHECK(neighbors_v1[0] == v0);

        auto neighbors_v2 = view.neighbors(v2);
        CHECK(neighbors_v2.size() == 1);
        CHECK(neighbors_v2[0] == v0);

        // v0 has no outgoing edges in reversed
        auto neighbors_v0 = view.neighbors(v0);
        CHECK(neighbors_v0.empty());
    }

    TEST_CASE("Reversed graph preserves base reference") {
        Graph<void> g;
        g.add_vertex();
        g.add_vertex();

        auto view = reversed(g);
        CHECK(&view.base() == &g);
    }

    TEST_CASE("Reversed graph with mixed edges") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);   // 0 -> 1 (reversed: 1 -> 0)
        g.add_edge(v1, v2, 2.0, EdgeType::Undirected); // 1 -- 2 (unchanged)

        auto view = reversed(g);
        auto edges = view.edges();

        CHECK(edges.size() == 2);

        // Check directed edge is reversed
        bool found_directed = false;
        bool found_undirected = false;
        for (const auto &e : edges) {
            if (e.type == EdgeType::Directed && e.source == 1 && e.target == 0) {
                found_directed = true;
            }
            if (e.type == EdgeType::Undirected) {
                found_undirected = true;
            }
        }
        CHECK(found_directed);
        CHECK(found_undirected);
    }
}

TEST_SUITE("Graph Views - FilteredGraphView") {

    TEST_CASE("Filtered empty graph") {
        Graph<void> g;
        auto view = FilteredGraphView<void>(g);

        CHECK(view.vertex_count() == 0);
        CHECK(view.edge_count() == 0);
    }

    TEST_CASE("Filter vertices by predicate") {
        Graph<void> g;
        for (int i = 0; i < 6; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);
        g.add_edge(1, 2, 1.0);
        g.add_edge(2, 3, 1.0);
        g.add_edge(3, 4, 1.0);
        g.add_edge(4, 5, 1.0);

        // Keep only even vertices
        auto view = filter_vertices_view(g, [](size_t v) { return v % 2 == 0; });

        CHECK(view.vertex_count() == 3); // 0, 2, 4
        CHECK(view.has_vertex(0));
        CHECK(!view.has_vertex(1));
        CHECK(view.has_vertex(2));
        CHECK(!view.has_vertex(3));
        CHECK(view.has_vertex(4));

        // Only edges between even vertices (none in this case)
        CHECK(view.edge_count() == 0);
    }

    TEST_CASE("Filter vertices preserves edges between kept vertices") {
        Graph<void> g;
        for (int i = 0; i < 4; i++)
            g.add_vertex();
        g.add_edge(0, 2, 1.0); // Both even - should be kept
        g.add_edge(1, 3, 1.0); // Both odd - should be removed
        g.add_edge(0, 1, 1.0); // Mixed - should be removed

        auto view = filter_vertices_view(g, [](size_t v) { return v % 2 == 0; });

        CHECK(view.vertex_count() == 2); // 0, 2
        CHECK(view.edge_count() == 1);   // Only 0-2
    }

    TEST_CASE("Filter edges by weight") {
        Graph<void> g;
        for (int i = 0; i < 4; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);
        g.add_edge(1, 2, 5.0);
        g.add_edge(2, 3, 10.0);

        // Keep only edges with weight > 3
        auto view = filter_edges_view(g, [](const EdgeDescriptor &e) { return e.weight > 3.0; });

        CHECK(view.vertex_count() == 4); // All vertices kept
        CHECK(view.edge_count() == 2);   // 1-2 and 2-3
    }

    TEST_CASE("Filter edges by type") {
        Graph<void> g;
        for (int i = 0; i < 3; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0, EdgeType::Directed);
        g.add_edge(1, 2, 1.0, EdgeType::Undirected);

        // Keep only directed edges
        auto view = filter_edges_view(g, [](const EdgeDescriptor &e) { return e.type == EdgeType::Directed; });

        CHECK(view.edge_count() == 1);
    }

    TEST_CASE("Combined vertex and edge filter") {
        Graph<void> g;
        for (int i = 0; i < 6; i++)
            g.add_vertex();
        g.add_edge(0, 2, 5.0);  // Both even, high weight
        g.add_edge(0, 4, 1.0);  // Both even, low weight
        g.add_edge(1, 3, 10.0); // Both odd, high weight

        // Keep even vertices AND high weight edges
        auto view = FilteredGraphView<void>(
            g, [](size_t v) { return v % 2 == 0; }, [](const EdgeDescriptor &e) { return e.weight > 3.0; });

        CHECK(view.vertex_count() == 3); // 0, 2, 4
        CHECK(view.edge_count() == 1);   // Only 0-2 (high weight between even)
    }

    TEST_CASE("Filtered view neighbors") {
        Graph<void> g;
        for (int i = 0; i < 5; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);
        g.add_edge(0, 2, 1.0);
        g.add_edge(0, 3, 1.0);
        g.add_edge(0, 4, 1.0);

        // Keep only even vertices
        auto view = filter_vertices_view(g, [](size_t v) { return v % 2 == 0; });

        auto neighbors = view.neighbors(0);
        CHECK(neighbors.size() == 2); // Only 2 and 4
    }

    TEST_CASE("Filter view preserves base reference") {
        Graph<void> g;
        g.add_vertex();

        auto view = filter_vertices_view(g, [](size_t) { return true; });
        CHECK(&view.base() == &g);
    }
}

TEST_SUITE("Graph Views - SubgraphView") {

    TEST_CASE("Subgraph of empty set") {
        Graph<void> g;
        for (int i = 0; i < 5; i++)
            g.add_vertex();

        std::unordered_set<size_t> empty_set;
        auto view = subgraph_view(g, empty_set);

        CHECK(view.vertex_count() == 0);
        CHECK(view.edge_count() == 0);
    }

    TEST_CASE("Subgraph with all vertices") {
        Graph<void> g;
        for (int i = 0; i < 3; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);
        g.add_edge(1, 2, 2.0);

        std::unordered_set<size_t> all = {0, 1, 2};
        auto view = subgraph_view(g, all);

        CHECK(view.vertex_count() == 3);
        CHECK(view.edge_count() == 2);
    }

    TEST_CASE("Subgraph with subset of vertices") {
        Graph<void> g;
        for (int i = 0; i < 5; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);
        g.add_edge(1, 2, 1.0);
        g.add_edge(2, 3, 1.0);
        g.add_edge(3, 4, 1.0);

        std::unordered_set<size_t> subset = {1, 2, 3};
        auto view = subgraph_view(g, subset);

        CHECK(view.vertex_count() == 3);
        CHECK(view.edge_count() == 2); // 1-2 and 2-3
    }

    TEST_CASE("Subgraph from vector") {
        Graph<void> g;
        for (int i = 0; i < 4; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);
        g.add_edge(2, 3, 1.0);

        std::vector<size_t> subset = {0, 1};
        auto view = subgraph_view(g, subset);

        CHECK(view.vertex_count() == 2);
        CHECK(view.edge_count() == 1); // Only 0-1
    }

    TEST_CASE("Subgraph preserves original vertex IDs") {
        Graph<void> g;
        for (int i = 0; i < 5; i++)
            g.add_vertex();

        std::unordered_set<size_t> subset = {2, 4};
        auto view = subgraph_view(g, subset);

        CHECK(view.has_vertex(2));
        CHECK(view.has_vertex(4));
        CHECK(!view.has_vertex(0));
        CHECK(!view.has_vertex(1));
        CHECK(!view.has_vertex(3));
    }

    TEST_CASE("Subgraph neighbors") {
        Graph<void> g;
        for (int i = 0; i < 5; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);
        g.add_edge(0, 2, 1.0);
        g.add_edge(0, 3, 1.0);
        g.add_edge(0, 4, 1.0);

        std::unordered_set<size_t> subset = {0, 2, 4};
        auto view = subgraph_view(g, subset);

        auto neighbors = view.neighbors(0);
        CHECK(neighbors.size() == 2); // Only 2 and 4 in subgraph
    }

    TEST_CASE("Subgraph edges only between included vertices") {
        Graph<void> g;
        for (int i = 0; i < 4; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0); // Excluded (1 not in subset)
        g.add_edge(0, 2, 2.0); // Included
        g.add_edge(2, 3, 3.0); // Excluded (3 not in subset)

        std::unordered_set<size_t> subset = {0, 2};
        auto view = subgraph_view(g, subset);

        CHECK(view.edge_count() == 1);
        auto edges = view.edges();
        CHECK(edges[0].weight == 2.0);
    }

    TEST_CASE("Subgraph preserves base and vertex_set") {
        Graph<void> g;
        for (int i = 0; i < 3; i++)
            g.add_vertex();

        std::unordered_set<size_t> subset = {0, 2};
        auto view = subgraph_view(g, subset);

        CHECK(&view.base() == &g);
        CHECK(view.vertex_set().size() == 2);
        CHECK(view.vertex_set().count(0));
        CHECK(view.vertex_set().count(2));
    }
}

TEST_SUITE("Graph Views - With Vertex Properties") {

    struct NodeData {
        std::string name;
        int value;
    };

    TEST_CASE("Reversed view with properties") {
        Graph<NodeData> g;
        auto v0 = g.add_vertex({"A", 1});
        auto v1 = g.add_vertex({"B", 2});
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);

        auto view = reversed(g);

        CHECK(view.vertex_count() == 2);
        CHECK(view[v0].name == "A");
        CHECK(view[v1].name == "B");

        // Edge should be reversed
        auto edges = view.edges();
        CHECK(edges.size() == 1);
        CHECK(edges[0].source == v1);
        CHECK(edges[0].target == v0);
    }

    TEST_CASE("Filtered view with properties") {
        Graph<NodeData> g;
        auto v0 = g.add_vertex({"A", 10});
        auto v1 = g.add_vertex({"B", 20});
        auto v2 = g.add_vertex({"C", 5});

        // Filter by property value
        auto view = FilteredGraphView<NodeData>(g, [&g](size_t v) { return g[v].value >= 10; });

        CHECK(view.vertex_count() == 2); // A and B
        CHECK(view.has_vertex(v0));
        CHECK(view.has_vertex(v1));
        CHECK(!view.has_vertex(v2));
    }

    TEST_CASE("Subgraph view with properties") {
        Graph<NodeData> g;
        auto v0 = g.add_vertex({"A", 1});
        auto v1 = g.add_vertex({"B", 2});
        auto v2 = g.add_vertex({"C", 3});

        std::unordered_set<size_t> subset = {v0, v2};
        auto view = subgraph_view(g, subset);

        CHECK(view.vertex_count() == 2);
        CHECK(view[v0].name == "A");
        CHECK(view[v2].name == "C");
    }
}

TEST_SUITE("Graph Views - Edge Cases") {

    TEST_CASE("View of single vertex graph") {
        Graph<void> g;
        auto v = g.add_vertex();

        auto reversed_view = reversed(g);
        CHECK(reversed_view.vertex_count() == 1);
        CHECK(reversed_view.edge_count() == 0);

        auto filtered_view = filter_vertices_view(g, [](size_t) { return true; });
        CHECK(filtered_view.vertex_count() == 1);

        std::unordered_set<size_t> all = {v};
        auto subgraph = subgraph_view(g, all);
        CHECK(subgraph.vertex_count() == 1);
    }

    TEST_CASE("Filter that removes all vertices") {
        Graph<void> g;
        for (int i = 0; i < 5; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);

        auto view = filter_vertices_view(g, [](size_t) { return false; });

        CHECK(view.vertex_count() == 0);
        CHECK(view.edge_count() == 0);
    }

    TEST_CASE("Filter that removes all edges") {
        Graph<void> g;
        for (int i = 0; i < 3; i++)
            g.add_vertex();
        g.add_edge(0, 1, 1.0);
        g.add_edge(1, 2, 1.0);

        auto view = filter_edges_view(g, [](const EdgeDescriptor &) { return false; });

        CHECK(view.vertex_count() == 3); // Vertices kept
        CHECK(view.edge_count() == 0);   // All edges removed
    }

    TEST_CASE("Subgraph with non-existent vertices in set") {
        Graph<void> g;
        g.add_vertex(); // v0
        g.add_vertex(); // v1

        // Include vertex that doesn't exist
        std::unordered_set<size_t> subset = {0, 1, 99};
        auto view = subgraph_view(g, subset);

        // Should only include existing vertices
        CHECK(view.has_vertex(0));
        CHECK(view.has_vertex(1));
        CHECK(!view.has_vertex(99)); // Doesn't exist in base graph
    }

    TEST_CASE("Views are lightweight (no copy)") {
        Graph<void> g;
        for (int i = 0; i < 1000; i++)
            g.add_vertex();

        // Views should just hold a reference
        auto rev = reversed(g);
        auto filt = filter_vertices_view(g, [](size_t) { return true; });

        // Verify they reference the same graph
        CHECK(&rev.base() == &g);
        CHECK(&filt.base() == &g);
    }

    TEST_CASE("Self-loop handling in reversed view") {
        Graph<void> g;
        auto v = g.add_vertex();
        g.add_edge(v, v, 1.0, EdgeType::Directed); // Self-loop

        auto view = reversed(g);

        // Self-loop reversed is still a self-loop
        auto edges = view.edges();
        CHECK(edges.size() == 1);
        CHECK(edges[0].source == v);
        CHECK(edges[0].target == v);
    }
}
