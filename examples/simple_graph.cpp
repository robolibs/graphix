#include "graphix/vertex/graph.hpp"
#include <iostream>

// Simple graph example: Create a small graph and query it

int main() {
    std::cout << "=== Simple Graph Example ===" << std::endl;
    std::cout << std::endl;

    // Create a graph without vertex properties
    graphix::vertex::Graph<void> g;

    // Add vertices
    std::cout << "Creating vertices..." << std::endl;
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();

    std::cout << "  Created " << g.vertex_count() << " vertices" << std::endl;
    std::cout << std::endl;

    // Add edges with weights
    std::cout << "Adding edges with weights..." << std::endl;
    auto e1 = g.add_edge(v1, v2, 1.0);
    auto e2 = g.add_edge(v2, v3, 2.0);
    auto e3 = g.add_edge(v3, v4, 1.5);
    auto e4 = g.add_edge(v1, v4, 5.0);

    std::cout << "  Added " << g.edge_count() << " edges" << std::endl;
    std::cout << std::endl;

    // Query graph structure
    std::cout << "Graph structure:" << std::endl;
    for (auto v : g.vertices()) {
        std::cout << "  Vertex " << v << " has degree " << g.degree(v);
        std::cout << ", neighbors: ";
        for (auto n : g.neighbors(v)) {
            std::cout << n << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // Query edges
    std::cout << "Edges in graph:" << std::endl;
    for (const auto &edge : g.edges()) {
        std::cout << "  Edge " << edge.id << ": ";
        std::cout << edge.source << " <-> " << edge.target;
        std::cout << " (weight: " << edge.weight << ")" << std::endl;
    }
    std::cout << std::endl;

    // Use Boost-style free functions
    std::cout << "Using Boost-style API:" << std::endl;
    std::cout << "  num_vertices: " << graphix::vertex::num_vertices(g) << std::endl;
    std::cout << "  num_edges: " << graphix::vertex::num_edges(g) << std::endl;
    std::cout << std::endl;

    // Edge queries
    std::cout << "Edge queries:" << std::endl;
    auto edge_opt = g.get_edge(v1, v2);
    if (edge_opt.has_value()) {
        auto eid = edge_opt.value();
        std::cout << "  Edge between v1 and v2: id=" << eid;
        std::cout << ", source=" << g.source(eid);
        std::cout << ", target=" << g.target(eid);
        std::cout << ", weight=" << g.get_weight(eid) << std::endl;
    }
    std::cout << std::endl;

    // Modify graph
    std::cout << "Removing edge between v1 and v4..." << std::endl;
    g.remove_edge(e4);
    std::cout << "  Edges remaining: " << g.edge_count() << std::endl;
    std::cout << "  Edge v1-v4 exists: " << (g.has_edge(v1, v4) ? "yes" : "no") << std::endl;

    return 0;
}
