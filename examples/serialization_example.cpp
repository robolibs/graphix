#include "graphix/vertex/graph.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

using namespace graphix::vertex;

// Simple 2D point for property demonstration
struct Point2D {
    double x;
    double y;

    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

void example1_simple_graph() {
    std::cout << "\n=== Example 1: Simple Graph (void properties) ===\n";

    // Create a simple graph without properties
    Graph<void> graph;
    auto v0 = graph.add_vertex();
    auto v1 = graph.add_vertex();
    auto v2 = graph.add_vertex();
    auto v3 = graph.add_vertex();

    // Add some edges
    graph.add_edge(v0, v1, 1.0);
    graph.add_edge(v1, v2, 2.0);
    graph.add_edge(v2, v3, 3.0);
    graph.add_edge(v3, v0, 4.0); // Create a cycle

    std::cout << "Original graph: " << graph.vertex_count() << " vertices, " << graph.edge_count() << " edges\n";

    // Save to DOT file
    graph.save_dot("simple_graph.dot");
    std::cout << "Saved to simple_graph.dot\n";

    // Load from DOT file
    auto loaded_graph = Graph<void>::load_dot("simple_graph.dot");
    std::cout << "Loaded graph: " << loaded_graph.vertex_count() << " vertices, " << loaded_graph.edge_count()
              << " edges\n";

    // Verify the graph structure
    std::cout << "Verification: ";
    if (loaded_graph.has_edge(v0, v1) && loaded_graph.has_edge(v1, v2) && loaded_graph.has_edge(v2, v3) &&
        loaded_graph.has_edge(v3, v0)) {
        std::cout << "All edges preserved!\n";
    } else {
        std::cout << "Something went wrong!\n";
    }

    // Show the DOT file content
    std::cout << "\nDOT file content:\n";
    std::cout << "-------------------\n";
    std::ifstream file("simple_graph.dot");
    std::cout << file.rdbuf();
    std::cout << "-------------------\n";
}

void example2_directed_graph() {
    std::cout << "\n=== Example 2: Directed Graph ===\n";

    Graph<void> graph;
    auto v0 = graph.add_vertex();
    auto v1 = graph.add_vertex();
    auto v2 = graph.add_vertex();

    // Add directed edges
    graph.add_edge(v0, v1, 1.0, EdgeType::Directed);
    graph.add_edge(v1, v2, 2.0, EdgeType::Directed);
    graph.add_edge(v2, v0, 3.0, EdgeType::Directed); // Create directed cycle

    std::cout << "Directed graph: " << graph.vertex_count() << " vertices, " << graph.edge_count() << " edges\n";

    graph.save_dot("directed_graph.dot");
    std::cout << "Saved to directed_graph.dot\n";

    auto loaded = Graph<void>::load_dot("directed_graph.dot");
    std::cout << "Loaded graph: " << loaded.vertex_count() << " vertices, " << loaded.edge_count() << " edges\n";

    // Verify directed edges
    std::cout << "Verification:\n";
    std::cout << "  v0 -> v1: " << (loaded.has_edge(v0, v1) ? "YES" : "NO") << "\n";
    std::cout << "  v1 -> v0: " << (loaded.has_edge(v1, v0) ? "YES" : "NO") << " (should be NO for directed)\n";

    // Show the DOT file content
    std::cout << "\nDOT file content:\n";
    std::cout << "-------------------\n";
    std::ifstream file("directed_graph.dot");
    std::cout << file.rdbuf();
    std::cout << "-------------------\n";
}

void example3_mixed_graph() {
    std::cout << "\n=== Example 3: Mixed Graph (Directed + Undirected) ===\n";

    Graph<void> graph;
    auto v0 = graph.add_vertex();
    auto v1 = graph.add_vertex();
    auto v2 = graph.add_vertex();
    auto v3 = graph.add_vertex();

    // Mix of directed and undirected edges
    graph.add_edge(v0, v1, 1.0, EdgeType::Undirected); // Bidirectional
    graph.add_edge(v1, v2, 2.0, EdgeType::Directed);   // One way
    graph.add_edge(v2, v3, 3.0, EdgeType::Undirected); // Bidirectional
    graph.add_edge(v3, v0, 4.0, EdgeType::Directed);   // One way

    std::cout << "Mixed graph: " << graph.vertex_count() << " vertices, " << graph.edge_count() << " edges\n";

    graph.save_dot("mixed_graph.dot");
    std::cout << "Saved to mixed_graph.dot\n";

    auto loaded = Graph<void>::load_dot("mixed_graph.dot");
    std::cout << "Loaded graph: " << loaded.vertex_count() << " vertices, " << loaded.edge_count() << " edges\n";

    // Verify mixed edges
    std::cout << "Verification:\n";
    std::cout << "  v0 <-> v1 (undirected): " << (loaded.has_edge(v0, v1) && loaded.has_edge(v1, v0) ? "YES" : "NO")
              << "\n";
    std::cout << "  v1 -> v2 (directed): " << (loaded.has_edge(v1, v2) && !loaded.has_edge(v2, v1) ? "YES" : "NO")
              << "\n";

    // Show the DOT file content
    std::cout << "\nDOT file content:\n";
    std::cout << "-------------------\n";
    std::ifstream file("mixed_graph.dot");
    std::cout << file.rdbuf();
    std::cout << "-------------------\n";
}

void example4_graph_with_properties() {
    std::cout << "\n=== Example 4: Graph with Vertex Properties ===\n";

    Graph<Point2D> graph;

    // Add vertices with 2D point properties
    auto v0 = graph.add_vertex(Point2D(0.0, 0.0));
    auto v1 = graph.add_vertex(Point2D(1.0, 0.0));
    auto v2 = graph.add_vertex(Point2D(1.0, 1.0));
    auto v3 = graph.add_vertex(Point2D(0.0, 1.0));

    // Connect them in a square
    graph.add_edge(v0, v1, 1.0);
    graph.add_edge(v1, v2, 1.0);
    graph.add_edge(v2, v3, 1.0);
    graph.add_edge(v3, v0, 1.0);
    graph.add_edge(v0, v2, 1.414); // Diagonal

    std::cout << "Graph with properties: " << graph.vertex_count() << " vertices, " << graph.edge_count() << " edges\n";

    // Define property writer (converts Point2D to string)
    auto writer = [](Graph<Point2D>::VertexId id, const Point2D &p) {
        std::ostringstream oss;
        oss << "(" << p.x << "," << p.y << ")";
        return oss.str();
    };

    graph.save_dot("property_graph.dot", writer);
    std::cout << "Saved to property_graph.dot\n";

    // Define property reader (converts string to Point2D)
    auto reader = [](const std::string &s) {
        Point2D p;
        // Parse "(x,y)" format
        size_t start = s.find('(');
        size_t comma = s.find(',');
        size_t end = s.find(')');
        if (start != std::string::npos && comma != std::string::npos && end != std::string::npos) {
            p.x = std::stod(s.substr(start + 1, comma - start - 1));
            p.y = std::stod(s.substr(comma + 1, end - comma - 1));
        }
        return p;
    };

    auto loaded = Graph<Point2D>::load_dot("property_graph.dot", reader);
    std::cout << "Loaded graph: " << loaded.vertex_count() << " vertices, " << loaded.edge_count() << " edges\n";

    // Verify properties
    std::cout << "Verification:\n";
    std::cout << "  Vertex 0 property: (" << loaded[v0].x << "," << loaded[v0].y << ")\n";
    std::cout << "  Vertex 1 property: (" << loaded[v1].x << "," << loaded[v1].y << ")\n";
    std::cout << "  Vertex 2 property: (" << loaded[v2].x << "," << loaded[v2].y << ")\n";

    // Show the DOT file content
    std::cout << "\nDOT file content:\n";
    std::cout << "-------------------\n";
    std::ifstream file("property_graph.dot");
    std::cout << file.rdbuf();
    std::cout << "-------------------\n";
}

void example5_visualization() {
    std::cout << "\n=== Example 5: Visualization with Graphviz (if available) ===\n";

    Graph<void> graph;
    auto v0 = graph.add_vertex();
    auto v1 = graph.add_vertex();
    auto v2 = graph.add_vertex();
    auto v3 = graph.add_vertex();
    auto v4 = graph.add_vertex();

    // Create a more interesting graph
    graph.add_edge(v0, v1, 1.0);
    graph.add_edge(v0, v2, 2.0);
    graph.add_edge(v1, v3, 3.0);
    graph.add_edge(v2, v3, 4.0);
    graph.add_edge(v3, v4, 5.0);

    graph.save_dot("visualization.dot");
    std::cout << "Saved to visualization.dot\n";
    std::cout << "\nTo visualize with Graphviz, run one of these commands:\n";
    std::cout << "  dot -Tpng visualization.dot -o visualization.png\n";
    std::cout << "  dot -Tsvg visualization.dot -o visualization.svg\n";
    std::cout << "  dot -Tpdf visualization.dot -o visualization.pdf\n";
    std::cout << "  neato -Tpng visualization.dot -o visualization_neato.png  (force-directed layout)\n";

    // Try to generate PNG if dot is available
    int result = system("which dot > /dev/null 2>&1");
    if (result == 0) {
        std::cout << "\nGraphviz 'dot' command found! Generating PNG...\n";
        system("dot -Tpng visualization.dot -o visualization.png");
        std::cout << "Generated visualization.png\n";
    } else {
        std::cout << "\nGraphviz 'dot' command not found. Install graphviz to generate images.\n";
    }
}

int main() {
    std::cout << "========================================\n";
    std::cout << "Graphix DOT Serialization Examples\n";
    std::cout << "========================================\n";

    // Run all examples
    example1_simple_graph();
    example2_directed_graph();
    example3_mixed_graph();
    example4_graph_with_properties();
    example5_visualization();

    std::cout << "\n========================================\n";
    std::cout << "All examples completed!\n";
    std::cout << "========================================\n";
    std::cout << "\nGenerated files:\n";
    std::cout << "  - simple_graph.dot\n";
    std::cout << "  - directed_graph.dot\n";
    std::cout << "  - mixed_graph.dot\n";
    std::cout << "  - property_graph.dot\n";
    std::cout << "  - visualization.dot\n";
    std::cout << "\nThese DOT files can be:\n";
    std::cout << "  1. Visualized with Graphviz (dot, neato, fdp, etc.)\n";
    std::cout << "  2. Loaded back into Graphix using load_dot()\n";
    std::cout << "  3. Edited manually and reloaded\n";
    std::cout << "  4. Used with other graph analysis tools that support DOT format\n";

    return 0;
}
