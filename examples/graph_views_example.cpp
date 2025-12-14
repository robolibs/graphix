// Graph Views Example
// Demonstrates lazy, zero-copy graph adapters
//
// Views vs Transformations:
// - Transformations (graph_transformations.hpp) CREATE new graphs
// - Views (graph_views.hpp) WRAP existing graphs without copying
//
// Features covered:
// 1. ReversedGraphView - View with reversed edge directions
// 2. FilteredGraphView - Lazy filtering of vertices/edges
// 3. SubgraphView - View of vertex subset (induced subgraph)

#include "graphix/vertex/algorithms/graph_views.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>
#include <string>

using namespace graphix::vertex;
using namespace graphix::vertex::views;

void print_separator(const std::string &title) {
    std::cout << "\n========================================\n";
    std::cout << title << "\n";
    std::cout << "========================================\n";
}

template <typename View> void print_view(const View &v, const std::string &name) {
    std::cout << name << ": " << v.vertex_count() << " vertices, " << v.edge_count() << " edges\n";
    std::cout << "  Vertices: ";
    for (auto vert : v.vertices()) {
        std::cout << vert << " ";
    }
    std::cout << "\n  Edges: ";
    for (const auto &e : v.edges()) {
        std::cout << e.source << (e.type == EdgeType::Directed ? "->" : "--") << e.target;
        std::cout << "(w=" << e.weight << ") ";
    }
    std::cout << "\n";
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

// Example 1: Reverse Dependencies
// Use case: Find "what depends on X" without creating a new graph
void example_reversed_view() {
    print_separator("Example 1: Reverse Dependencies (ReversedGraphView)");

    std::cout << "Scenario: Package dependency analysis\n";
    std::cout << "Original: A depends on B means A -> B\n";
    std::cout << "Reversed view: Find what depends on each package\n\n";

    Graph<void> deps;
    auto app = deps.add_vertex();    // 0: app
    auto web = deps.add_vertex();    // 1: web-framework
    auto db = deps.add_vertex();     // 2: database
    auto utils = deps.add_vertex();  // 3: utils
    auto logger = deps.add_vertex(); // 4: logger

    // Dependencies (A -> B means A depends on B)
    deps.add_edge(app, web, 1.0, EdgeType::Directed);
    deps.add_edge(app, db, 1.0, EdgeType::Directed);
    deps.add_edge(web, utils, 1.0, EdgeType::Directed);
    deps.add_edge(web, logger, 1.0, EdgeType::Directed);
    deps.add_edge(db, utils, 1.0, EdgeType::Directed);
    deps.add_edge(db, logger, 1.0, EdgeType::Directed);

    print_graph(deps, "Original dependency graph");

    // Create reversed view (zero-copy!)
    auto reverse_view = reversed(deps);
    print_view(reverse_view, "Reversed view (what depends on X?)");

    std::cout << "\nQuery: What depends on utils (vertex 3)?\n";
    auto dependents = reverse_view.neighbors(utils);
    std::cout << "  Dependents: ";
    for (auto d : dependents) {
        std::cout << d << " ";
    }
    std::cout << "(web=1, db=2)\n";

    std::cout << "\nNote: No new graph created - view wraps original!\n";
    std::cout << "  &reverse_view.base() == &deps: " << (&reverse_view.base() == &deps ? "true" : "false") << "\n";
}

// Example 2: Security Zone Filtering
// Use case: View only nodes in a security zone without copying
void example_filtered_view() {
    print_separator("Example 2: Security Zone Analysis (FilteredGraphView)");

    std::cout << "Scenario: Network with security zones\n";
    std::cout << "Vertices 0-2: Public zone, Vertices 3-5: Private zone\n\n";

    Graph<void> network;
    // Public zone (0, 1, 2)
    for (int i = 0; i < 3; i++)
        network.add_vertex();
    // Private zone (3, 4, 5)
    for (int i = 0; i < 3; i++)
        network.add_vertex();

    // Public connections
    network.add_edge(0, 1, 10.0);
    network.add_edge(1, 2, 10.0);
    // Private connections
    network.add_edge(3, 4, 100.0);
    network.add_edge(4, 5, 100.0);
    // Cross-zone connections (should be filtered out)
    network.add_edge(2, 3, 1.0); // Gateway

    print_graph(network, "Full network");

    // View only public zone
    auto public_view = filter_vertices_view(network, [](size_t v) { return v < 3; });
    print_view(public_view, "Public zone view (vertices < 3)");

    // View only high-bandwidth links
    auto high_bw_view = filter_edges_view(network, [](const EdgeDescriptor &e) { return e.weight >= 10.0; });
    print_view(high_bw_view, "High-bandwidth view (weight >= 10)");

    std::cout << "\nBoth views reference same underlying graph (zero-copy)\n";
}

// Example 3: Community Analysis
// Use case: Analyze a specific community without extracting it
void example_subgraph_view() {
    print_separator("Example 3: Community Analysis (SubgraphView)");

    std::cout << "Scenario: Social network with identified communities\n\n";

    Graph<void> social;
    for (int i = 0; i < 9; i++)
        social.add_vertex();

    // Community A (0, 1, 2) - tightly connected
    social.add_edge(0, 1, 1.0);
    social.add_edge(1, 2, 1.0);
    social.add_edge(0, 2, 1.0);

    // Community B (3, 4, 5) - tightly connected
    social.add_edge(3, 4, 1.0);
    social.add_edge(4, 5, 1.0);
    social.add_edge(3, 5, 1.0);

    // Community C (6, 7, 8) - tightly connected
    social.add_edge(6, 7, 1.0);
    social.add_edge(7, 8, 1.0);
    social.add_edge(6, 8, 1.0);

    // Inter-community bridges
    social.add_edge(2, 3, 1.0); // A-B bridge
    social.add_edge(5, 6, 1.0); // B-C bridge

    print_graph(social, "Full social network");

    // View Community A
    std::unordered_set<size_t> community_a = {0, 1, 2};
    auto view_a = subgraph_view(social, community_a);
    print_view(view_a, "Community A view");

    // View Community B
    std::vector<size_t> community_b = {3, 4, 5};
    auto view_b = subgraph_view(social, community_b);
    print_view(view_b, "Community B view");

    std::cout << "\nAnalyze community density without copying:\n";
    std::cout << "  Community A: " << view_a.vertex_count() << " vertices, " << view_a.edge_count() << " edges\n";
    std::cout << "  Community B: " << view_b.vertex_count() << " vertices, " << view_b.edge_count() << " edges\n";
    std::cout << "  Both are complete graphs (3 edges for 3 vertices)\n";
}

// Example 4: Multi-level Filtering
// Use case: Chain multiple views for complex queries
void example_chained_views() {
    print_separator("Example 4: Multi-level Analysis (Chained Views)");

    std::cout << "Scenario: Transportation network analysis\n";
    std::cout << "Find high-capacity routes in the eastern region\n\n";

    Graph<void> transport;
    // Western cities (0-2)
    for (int i = 0; i < 3; i++)
        transport.add_vertex();
    // Eastern cities (3-5)
    for (int i = 0; i < 3; i++)
        transport.add_vertex();

    // Western routes (low capacity)
    transport.add_edge(0, 1, 50.0);
    transport.add_edge(1, 2, 40.0);

    // Eastern routes (mixed capacity)
    transport.add_edge(3, 4, 200.0); // High capacity
    transport.add_edge(4, 5, 30.0);  // Low capacity
    transport.add_edge(3, 5, 150.0); // High capacity

    // Cross-region (medium)
    transport.add_edge(2, 3, 80.0);

    print_graph(transport, "Full transport network");

    // First filter: Eastern region only (vertices >= 3)
    auto eastern_view = filter_vertices_view(transport, [](size_t v) { return v >= 3; });
    print_view(eastern_view, "Eastern region view");

    // Second filter: High capacity only (weight >= 100)
    // Note: We create this from the original graph but with combined predicates
    auto high_cap_eastern = FilteredGraphView<void>(
        transport, [](size_t v) { return v >= 3; }, [](const EdgeDescriptor &e) { return e.weight >= 100.0; });

    print_view(high_cap_eastern, "High-capacity eastern routes");

    std::cout << "\nResult: 2 high-capacity routes in eastern region\n";
}

// Example 5: Algorithm Compatibility
// Use case: Run algorithms on views (they expose same interface)
void example_algorithm_on_view() {
    print_separator("Example 5: Algorithms on Views");

    std::cout << "Scenario: Count neighbors through different views\n\n";

    Graph<void> g;
    for (int i = 0; i < 5; i++)
        g.add_vertex();
    g.add_edge(0, 1, 1.0, EdgeType::Directed);   // 0 -> 1
    g.add_edge(0, 2, 1.0, EdgeType::Directed);   // 0 -> 2
    g.add_edge(0, 3, 1.0, EdgeType::Undirected); // 0 -- 3
    g.add_edge(0, 4, 1.0, EdgeType::Undirected); // 0 -- 4

    std::cout << "Original graph neighbors of vertex 0:\n";
    auto n0 = g.neighbors(0);
    std::cout << "  Count: " << n0.size() << " (1, 2, 3, 4)\n";

    auto rev = reversed(g);
    std::cout << "\nReversed view neighbors of vertex 0:\n";
    auto n0_rev = rev.neighbors(0);
    std::cout << "  Count: " << n0_rev.size() << " (only undirected: 3, 4)\n";

    std::cout << "\nIn reversed view, vertices 1 and 2 now point TO 0:\n";
    std::cout << "  Neighbors of 1: ";
    for (auto n : rev.neighbors(1))
        std::cout << n << " ";
    std::cout << "(0)\n";
    std::cout << "  Neighbors of 2: ";
    for (auto n : rev.neighbors(2))
        std::cout << n << " ";
    std::cout << "(0)\n";
}

int main() {
    std::cout << "Graph Views Examples\n";
    std::cout << "====================\n";
    std::cout << "Demonstrating lazy, zero-copy graph adapters\n";

    example_reversed_view();
    example_filtered_view();
    example_subgraph_view();
    example_chained_views();
    example_algorithm_on_view();

    print_separator("Summary");
    std::cout << "Graph View Types:\n\n";
    std::cout << "  ReversedGraphView  - Reversed edge directions (directed only)\n";
    std::cout << "  FilteredGraphView  - Lazy vertex/edge filtering by predicate\n";
    std::cout << "  SubgraphView       - Induced subgraph from vertex set\n";
    std::cout << "\nHelper functions:\n";
    std::cout << "  reversed(g)              - Create reversed view\n";
    std::cout << "  filter_vertices_view(g, pred) - Filter by vertex predicate\n";
    std::cout << "  filter_edges_view(g, pred)    - Filter by edge predicate\n";
    std::cout << "  subgraph_view(g, vertices)    - Create subgraph view\n";
    std::cout << "\nKey benefits:\n";
    std::cout << "  - Zero-copy: Views wrap original graph\n";
    std::cout << "  - Lazy evaluation: Predicates evaluated on access\n";
    std::cout << "  - Same interface: Works with algorithms expecting graph-like objects\n";

    return 0;
}
