// Graph Transformations Example
// Demonstrates utility functions for modifying and deriving new graphs
//
// Features covered:
// 1. Transpose - Reverse edge directions
// 2. Induced Subgraph - Extract subgraph from vertex set
// 3. Graph Union - Combine two graphs
// 4. Graph Intersection - Find common structure
// 5. Graph Complement - Invert edge structure
// 6. Filter Functions - Select vertices/edges by predicate

#include "graphix/vertex/algorithms/graph_transformations.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>
#include <string>

using namespace graphix::vertex;
using namespace graphix::vertex::algorithms;

void print_separator(const std::string &title) {
    std::cout << "\n========================================\n";
    std::cout << title << "\n";
    std::cout << "========================================\n";
}

void print_graph(const Graph<void> &g, const std::string &name) {
    std::cout << name << ": " << g.vertex_count() << " vertices, " << g.edge_count() << " edges\n";
    std::cout << "  Vertices: ";
    for (auto v : g.vertices()) {
        std::cout << v << " ";
    }
    std::cout << "\n  Edges: ";
    for (const auto &e : g.edges()) {
        std::cout << e.source << (e.type == EdgeType::Directed ? "->" : "--") << e.target;
        std::cout << "(w=" << e.weight << ") ";
    }
    std::cout << "\n";
}

// Example 1: Dependency Graph Reversal
// Use case: Build systems need to find "what depends on X" (reverse deps)
void example_transpose() {
    print_separator("Example 1: Dependency Graph Reversal (Transpose)");

    std::cout << "Scenario: Build system dependency graph\n";
    std::cout << "Original graph shows: A depends on B means A -> B\n\n";

    Graph<void> deps;
    auto app = deps.add_vertex();      // 0: application
    auto lib_a = deps.add_vertex();    // 1: lib_a
    auto lib_b = deps.add_vertex();    // 2: lib_b
    auto lib_core = deps.add_vertex(); // 3: lib_core

    // app depends on lib_a and lib_b
    deps.add_edge(app, lib_a, 1.0, EdgeType::Directed);
    deps.add_edge(app, lib_b, 1.0, EdgeType::Directed);
    // lib_a and lib_b both depend on lib_core
    deps.add_edge(lib_a, lib_core, 1.0, EdgeType::Directed);
    deps.add_edge(lib_b, lib_core, 1.0, EdgeType::Directed);

    print_graph(deps, "Original (what does X depend on?)");

    auto reverse_deps = transpose(deps);
    print_graph(reverse_deps, "Transposed (what depends on X?)");

    std::cout << "\nInterpretation:\n";
    std::cout << "  - lib_core (3) is now depended upon by lib_a (1) and lib_b (2)\n";
    std::cout << "  - Useful for: impact analysis, cache invalidation\n";
}

// Example 2: Community Extraction
// Use case: Extract a subnetwork for focused analysis
void example_subgraph() {
    print_separator("Example 2: Community Extraction (Induced Subgraph)");

    std::cout << "Scenario: Social network - extract a friend group\n\n";

    Graph<void> social;
    // Create a larger network
    for (int i = 0; i < 8; i++) {
        social.add_vertex();
    }

    // Group A (0, 1, 2) - tightly connected
    social.add_edge(0, 1, 1.0);
    social.add_edge(1, 2, 1.0);
    social.add_edge(0, 2, 1.0);

    // Group B (5, 6, 7) - tightly connected
    social.add_edge(5, 6, 1.0);
    social.add_edge(6, 7, 1.0);
    social.add_edge(5, 7, 1.0);

    // Bridge connections
    social.add_edge(2, 3, 1.0);
    social.add_edge(3, 4, 1.0);
    social.add_edge(4, 5, 1.0);

    print_graph(social, "Full social network");

    // Extract just Group A
    std::unordered_set<size_t> group_a = {0, 1, 2};
    auto subgraph_a = induced_subgraph(social, group_a);
    print_graph(subgraph_a, "Group A subgraph");

    std::cout << "\nNote: Induced subgraph preserves all edges between selected vertices\n";
}

// Example 3: Merging Knowledge Graphs
// Use case: Combine data from multiple sources
void example_union() {
    print_separator("Example 3: Merging Knowledge Graphs (Union)");

    std::cout << "Scenario: Merge two knowledge bases\n\n";

    Graph<void> kb1;            // Knowledge base 1
    auto a1 = kb1.add_vertex(); // concept A
    auto b1 = kb1.add_vertex(); // concept B
    kb1.add_edge(a1, b1, 1.0);  // A relates to B

    Graph<void> kb2;            // Knowledge base 2
    auto c2 = kb2.add_vertex(); // concept C
    auto d2 = kb2.add_vertex(); // concept D
    kb2.add_edge(c2, d2, 2.0);  // C relates to D

    print_graph(kb1, "Knowledge Base 1");
    print_graph(kb2, "Knowledge Base 2");

    auto merged = graph_union(kb1, kb2);
    print_graph(merged.graph, "Merged graph");

    std::cout << "\nVertex ID mappings:\n";
    std::cout << "  KB1 vertex 0 -> merged vertex " << merged.g1_mapping.at(0) << "\n";
    std::cout << "  KB1 vertex 1 -> merged vertex " << merged.g1_mapping.at(1) << "\n";
    std::cout << "  KB2 vertex 0 -> merged vertex " << merged.g2_mapping.at(0) << "\n";
    std::cout << "  KB2 vertex 1 -> merged vertex " << merged.g2_mapping.at(1) << "\n";
}

// Example 4: Finding Common Infrastructure
// Use case: Find shared components between two systems
void example_intersection() {
    print_separator("Example 4: Finding Common Infrastructure (Intersection)");

    std::cout << "Scenario: Find shared servers between two deployments\n\n";

    Graph<void> prod; // Production deployment
    for (int i = 0; i < 5; i++)
        prod.add_vertex(); // servers 0-4
    prod.add_edge(0, 1, 1.0);
    prod.add_edge(1, 2, 1.0);
    prod.add_edge(2, 3, 1.0);
    prod.add_edge(3, 4, 1.0);

    Graph<void> staging; // Staging deployment
    for (int i = 0; i < 4; i++)
        staging.add_vertex(); // servers 0-3
    staging.add_edge(0, 1, 1.0);
    staging.add_edge(1, 2, 1.0);
    staging.add_edge(2, 3, 1.0);

    print_graph(prod, "Production (servers 0-4)");
    print_graph(staging, "Staging (servers 0-3)");

    auto common = graph_intersection(prod, staging);
    print_graph(common, "Common infrastructure");

    std::cout << "\nServers 0-3 and their connections exist in both environments\n";
}

// Example 5: Finding Missing Connections
// Use case: Recommendation system - suggest new connections
void example_complement() {
    print_separator("Example 5: Finding Missing Connections (Complement)");

    std::cout << "Scenario: Friend suggestions - find who ISN'T connected\n\n";

    Graph<void> friends;
    auto alice = friends.add_vertex(); // 0
    auto bob = friends.add_vertex();   // 1
    auto carol = friends.add_vertex(); // 2
    auto dave = friends.add_vertex();  // 3

    // Existing friendships
    friends.add_edge(alice, bob, 1.0);  // Alice-Bob
    friends.add_edge(bob, carol, 1.0);  // Bob-Carol
    friends.add_edge(carol, dave, 1.0); // Carol-Dave

    print_graph(friends, "Current friendships");

    auto non_friends = complement(friends);
    print_graph(non_friends, "Potential new connections (complement)");

    std::cout << "\nPotential friend suggestions:\n";
    std::cout << "  - Alice (0) and Carol (2) have a mutual friend (Bob)\n";
    std::cout << "  - Bob (1) and Dave (3) have a mutual friend (Carol)\n";
    std::cout << "  - Alice (0) and Dave (3) are 3 hops apart\n";
}

// Example 6: Network Filtering
// Use case: Focus analysis on high-traffic routes
void example_filtering() {
    print_separator("Example 6: Network Analysis (Filter Functions)");

    std::cout << "Scenario: Network traffic - filter by bandwidth\n\n";

    Graph<void> network;
    for (int i = 0; i < 5; i++)
        network.add_vertex(); // routers

    // Add links with bandwidth (weight = Gbps)
    network.add_edge(0, 1, 10.0);  // 10 Gbps
    network.add_edge(1, 2, 1.0);   // 1 Gbps
    network.add_edge(2, 3, 10.0);  // 10 Gbps
    network.add_edge(3, 4, 1.0);   // 1 Gbps
    network.add_edge(0, 4, 100.0); // 100 Gbps backbone

    print_graph(network, "Full network");

    // Filter to high-bandwidth links only (> 5 Gbps)
    auto high_bandwidth =
        filter_edges(network, [](const EdgeDescriptor &e, const Graph<void> &) { return e.weight > 5.0; });
    print_graph(high_bandwidth, "High-bandwidth links (> 5 Gbps)");

    // Filter to core routers (even IDs in this example)
    auto core_routers = filter_vertices(network, [](size_t v, const Graph<void> &) {
        return v % 2 == 0; // vertices 0, 2, 4
    });
    print_graph(core_routers, "Core router subnetwork (even IDs)");

    std::cout << "\nNote: filter_edges keeps all vertices but only matching edges\n";
    std::cout << "      filter_vertices creates induced subgraph of matching vertices\n";
}

int main() {
    std::cout << "Graph Transformations Examples\n";
    std::cout << "==============================\n";
    std::cout << "Demonstrating utility functions for graph manipulation\n";

    example_transpose();
    example_subgraph();
    example_union();
    example_intersection();
    example_complement();
    example_filtering();

    print_separator("Summary");
    std::cout << "Graph transformation functions:\n\n";
    std::cout << "  transpose()        - Reverse all directed edges\n";
    std::cout << "  induced_subgraph() - Extract subgraph from vertex set\n";
    std::cout << "  edge_subgraph()    - Extract with specific edges\n";
    std::cout << "  graph_union()      - Combine two graphs (disjoint)\n";
    std::cout << "  graph_intersection() - Find common vertices/edges\n";
    std::cout << "  complement()       - Invert edge structure\n";
    std::cout << "  filter_vertices()  - Select vertices by predicate\n";
    std::cout << "  filter_edges()     - Select edges by predicate\n";
    std::cout << "\nAll functions return NEW graphs (immutable pattern)\n";

    return 0;
}
