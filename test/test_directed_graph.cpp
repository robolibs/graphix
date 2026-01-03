#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

using namespace graphix::vertex;

TEST_CASE("Directed Graph - Basic directed edges") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    // Add directed edges: 0 -> 1 -> 2
    g.add_edge(v0, v1, 1.0, EdgeType::Directed);
    g.add_edge(v1, v2, 1.0, EdgeType::Directed);

    SUBCASE("Directed edge only exists in one direction") {
        CHECK(g.has_edge(v0, v1) == true);
        CHECK(g.has_edge(v1, v0) == false); // Reverse doesn't exist

        CHECK(g.has_edge(v1, v2) == true);
        CHECK(g.has_edge(v2, v1) == false);
    }

    SUBCASE("Neighbors only include outgoing edges") {
        auto neighbors_v0 = g.neighbors(v0);
        CHECK(neighbors_v0.size() == 1);
        CHECK(neighbors_v0[0] == v1);

        auto neighbors_v1 = g.neighbors(v1);
        CHECK(neighbors_v1.size() == 1);
        CHECK(neighbors_v1[0] == v2);

        // v2 has no outgoing edges
        auto neighbors_v2 = g.neighbors(v2);
        CHECK(neighbors_v2.empty());
    }

    SUBCASE("Degree counts only outgoing edges for directed") {
        CHECK(g.degree(v0) == 1);
        CHECK(g.degree(v1) == 1);
        CHECK(g.degree(v2) == 0); // No outgoing edges
    }
}

TEST_CASE("Mixed Graph - Directed and undirected edges") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    // Create a mixed graph:
    // 0 <-> 1 (undirected)
    // 1 ->  2 (directed)
    // 2 <-> 3 (undirected)
    g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
    g.add_edge(v1, v2, 2.0, EdgeType::Directed);
    g.add_edge(v2, v3, 3.0, EdgeType::Undirected);

    SUBCASE("Undirected edges work both ways") {
        CHECK(g.has_edge(v0, v1) == true);
        CHECK(g.has_edge(v1, v0) == true); // Bidirectional

        CHECK(g.has_edge(v2, v3) == true);
        CHECK(g.has_edge(v3, v2) == true); // Bidirectional
    }

    SUBCASE("Directed edge only works one way") {
        CHECK(g.has_edge(v1, v2) == true);
        CHECK(g.has_edge(v2, v1) == false); // Unidirectional
    }

    SUBCASE("Neighbors reflect edge directionality") {
        auto neighbors_v1 = g.neighbors(v1);
        CHECK(neighbors_v1.size() == 2); // v0 (undirected) and v2 (directed)

        auto neighbors_v2 = g.neighbors(v2);
        CHECK(neighbors_v2.size() == 1); // Only v3 (undirected), NOT v1
    }
}

TEST_CASE("Directed Graph - Edge iteration") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();

    g.add_edge(v0, v1, 5.0, EdgeType::Directed);

    auto all_edges = g.edges();

    SUBCASE("Directed edge appears once in edge list") {
        CHECK(all_edges.size() == 1);
        CHECK(all_edges[0].source == v0);
        CHECK(all_edges[0].target == v1);
        CHECK(all_edges[0].weight == 5.0);
        CHECK(all_edges[0].type == EdgeType::Directed);
    }
}

TEST_CASE("Undirected Graph - Edge iteration (backward compatibility)") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();

    // Default is undirected
    g.add_edge(v0, v1, 5.0);

    auto all_edges = g.edges();

    SUBCASE("Undirected edge appears once in edge list") {
        CHECK(all_edges.size() == 1);
        CHECK(all_edges[0].weight == 5.0);
        CHECK(all_edges[0].type == EdgeType::Undirected);
    }

    SUBCASE("Edge works in both directions") {
        CHECK(g.has_edge(v0, v1) == true);
        CHECK(g.has_edge(v1, v0) == true);
    }
}

TEST_CASE("Directed Graph - Complete directed graph") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    // All edges directed
    g.add_edge(v0, v1, 1.0, EdgeType::Directed);
    g.add_edge(v0, v2, 2.0, EdgeType::Directed);
    g.add_edge(v1, v2, 3.0, EdgeType::Directed);

    CHECK(g.edge_count() == 3);

    // Check all edges exist in correct direction
    CHECK(g.has_edge(v0, v1) == true);
    CHECK(g.has_edge(v0, v2) == true);
    CHECK(g.has_edge(v1, v2) == true);

    // Check reverse edges don't exist
    CHECK(g.has_edge(v1, v0) == false);
    CHECK(g.has_edge(v2, v0) == false);
    CHECK(g.has_edge(v2, v1) == false);
}

TEST_CASE("Mixed Graph - Edge removal") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    auto e01 = g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
    auto e12 = g.add_edge(v1, v2, 2.0, EdgeType::Directed);

    SUBCASE("Remove directed edge") {
        g.remove_edge(e12);

        CHECK(g.has_edge(v1, v2) == false);
        CHECK(g.has_edge(v0, v1) == true); // Other edge unaffected
        CHECK(g.edge_count() == 1);
    }

    SUBCASE("Remove undirected edge") {
        g.remove_edge(e01);

        CHECK(g.has_edge(v0, v1) == false);
        CHECK(g.has_edge(v1, v0) == false); // Both directions gone
        CHECK(g.has_edge(v1, v2) == true);  // Other edge unaffected
        CHECK(g.edge_count() == 1);
    }
}

TEST_CASE("Directed Graph - Cycle") {
    Graph<void> g;

    // Create a directed cycle: 0 -> 1 -> 2 -> 0
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    g.add_edge(v0, v1, 1.0, EdgeType::Directed);
    g.add_edge(v1, v2, 1.0, EdgeType::Directed);
    g.add_edge(v2, v0, 1.0, EdgeType::Directed);

    SUBCASE("All edges go in cycle direction") {
        CHECK(g.has_edge(v0, v1) == true);
        CHECK(g.has_edge(v1, v2) == true);
        CHECK(g.has_edge(v2, v0) == true);
    }

    SUBCASE("No edges go backwards") {
        CHECK(g.has_edge(v1, v0) == false);
        CHECK(g.has_edge(v2, v1) == false);
        CHECK(g.has_edge(v0, v2) == false);
    }

    SUBCASE("Each vertex has degree 1") {
        CHECK(g.degree(v0) == 1);
        CHECK(g.degree(v1) == 1);
        CHECK(g.degree(v2) == 1);
    }
}

TEST_CASE("Mixed Graph - With properties") {
    struct Node {
        std::string name;
    };

    Graph<Node> g;

    auto alice = g.add_vertex({"Alice"});
    auto bob = g.add_vertex({"Bob"});
    auto charlie = g.add_vertex({"Charlie"});

    // Alice follows Bob (directed)
    g.add_edge(alice, bob, 1.0, EdgeType::Directed);

    // Bob and Charlie are friends (undirected)
    g.add_edge(bob, charlie, 1.0, EdgeType::Undirected);

    CHECK(g.has_edge(alice, bob) == true);   // Alice -> Bob
    CHECK(g.has_edge(bob, alice) == false);  // Bob doesn't follow Alice
    CHECK(g.has_edge(bob, charlie) == true); // Bob <-> Charlie
    CHECK(g.has_edge(charlie, bob) == true); // Charlie <-> Bob

    // Properties still work
    CHECK(g[alice].name == "Alice");
    CHECK(g[bob].name == "Bob");
    CHECK(g[charlie].name == "Charlie");
}

TEST_CASE("Directed Graph - Self loop") {
    Graph<void> g;

    auto v0 = g.add_vertex();

    // Self-directed edge
    g.add_edge(v0, v0, 1.0, EdgeType::Directed);

    CHECK(g.has_edge(v0, v0) == true);
    CHECK(g.degree(v0) == 1);
    CHECK(g.edge_count() == 1);
}

TEST_CASE("Mixed Graph - Parallel edges with different types") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();

    // Add both directed and undirected edge between same vertices
    auto e_dir = g.add_edge(v0, v1, 1.0, EdgeType::Directed);
    auto e_undir = g.add_edge(v0, v1, 2.0, EdgeType::Undirected);

    CHECK(e_dir != e_undir); // Different edge IDs
    CHECK(g.edge_count() == 2);

    // Both edges should be present
    auto neighbors = g.neighbors(v0);
    CHECK(neighbors.size() == 2); // Both edges from v0 to v1
}
