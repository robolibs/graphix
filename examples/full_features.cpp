#include "graphix/vertex/algorithms/dijkstra.hpp"
#include "graphix/vertex/graph.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

// Full features example: Demonstrates all graph capabilities

struct Point2D {
    double x, y;
    std::string label;

    Point2D(double x = 0, double y = 0, const std::string &lbl = "") : x(x), y(y), label(lbl) {}

    double distance_to(const Point2D &other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

void print_separator() { std::cout << "----------------------------------------" << std::endl; }

int main() {
    std::cout << "=== Full Features Demonstration ===" << std::endl;
    print_separator();

    // 1. CREATE GRAPH WITH PROPERTIES
    std::cout << "1. Creating graph with Point2D vertex properties..." << std::endl;
    graphix::vertex::Graph<Point2D> g;

    auto a = g.add_vertex(Point2D(0, 0, "A"));
    auto b = g.add_vertex(Point2D(3, 0, "B"));
    auto c = g.add_vertex(Point2D(3, 4, "C"));
    auto d = g.add_vertex(Point2D(0, 4, "D"));
    auto e = g.add_vertex(Point2D(1.5, 2, "E"));

    std::cout << "   Created " << graphix::vertex::num_vertices(g) << " vertices" << std::endl;
    print_separator();

    // 2. ADD EDGES WITH WEIGHTS
    std::cout << "2. Adding weighted edges (using Euclidean distances)..." << std::endl;
    auto e_ab = g.add_edge(a, b, g[a].distance_to(g[b]));
    auto e_bc = g.add_edge(b, c, g[b].distance_to(g[c]));
    auto e_cd = g.add_edge(c, d, g[c].distance_to(g[d]));
    auto e_da = g.add_edge(d, a, g[d].distance_to(g[a]));
    auto e_ae = g.add_edge(a, e, g[a].distance_to(g[e]));
    auto e_be = g.add_edge(b, e, g[b].distance_to(g[e]));
    auto e_ce = g.add_edge(c, e, g[c].distance_to(g[e]));
    auto e_de = g.add_edge(d, e, g[d].distance_to(g[e]));

    std::cout << "   Added " << graphix::vertex::num_edges(g) << " edges" << std::endl;
    print_separator();

    // 3. ITERATE VERTICES AND PROPERTIES
    std::cout << "3. Vertex properties and degrees:" << std::endl;
    for (auto v : graphix::vertex::vertices(g)) {
        const auto &point = g[v];
        std::cout << "   " << point.label << " at (" << point.x << ", " << point.y << ")";
        std::cout << " - degree: " << graphix::vertex::degree(v, g) << std::endl;
    }
    print_separator();

    // 4. ITERATE EDGES
    std::cout << "4. Edge information:" << std::endl;
    for (const auto &edge : graphix::vertex::edges(g)) {
        auto src = graphix::vertex::source(edge.id, g);
        auto tgt = graphix::vertex::target(edge.id, g);
        std::cout << "   " << g[src].label << " <-> " << g[tgt].label;
        std::cout << " (weight: " << std::fixed << std::setprecision(2) << edge.weight << ")" << std::endl;
    }
    print_separator();

    // 5. QUERY NEIGHBORS
    std::cout << "5. Neighbors of vertex E:" << std::endl;
    std::cout << "   ";
    for (auto neighbor : graphix::vertex::neighbors(e, g)) {
        std::cout << g[neighbor].label << " ";
    }
    std::cout << std::endl;
    print_separator();

    // 6. EDGE QUERIES
    std::cout << "6. Edge query examples:" << std::endl;

    auto edge_result = graphix::vertex::get_edge(a, b, g);
    if (edge_result.has_value()) {
        auto eid = edge_result.value();
        std::cout << "   Edge A-B exists: id=" << eid;
        std::cout << ", weight=" << std::fixed << std::setprecision(2);
        std::cout << g.get_weight(eid) << std::endl;
    }

    auto [eid_ac, exists_ac] = graphix::vertex::edge(a, c, g);
    std::cout << "   Direct edge A-C exists: " << (exists_ac ? "yes" : "no") << std::endl;
    print_separator();

    // 7. SHORTEST PATH ALGORITHM
    std::cout << "7. Shortest path from A to C:" << std::endl;
    auto path_result = graphix::vertex::algorithms::dijkstra(g, a, c);

    if (path_result.found) {
        std::cout << "   Distance: " << std::fixed << std::setprecision(2);
        std::cout << path_result.distance << std::endl;
        std::cout << "   Path: ";
        for (size_t i = 0; i < path_result.path.size(); ++i) {
            std::cout << g[path_result.path[i]].label;
            if (i < path_result.path.size() - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;
    }
    print_separator();

    // 8. MODIFY EDGE WEIGHT
    std::cout << "8. Modifying edge weight (A-B from " << std::fixed << std::setprecision(2);
    std::cout << g.get_weight(e_ab) << " to 10.0)..." << std::endl;

    g.set_weight(e_ab, 10.0);
    std::cout << "   New weight: " << g.get_weight(e_ab) << std::endl;

    // Recompute shortest path
    auto new_path = graphix::vertex::algorithms::dijkstra(g, a, c);
    std::cout << "   New shortest path A->C: ";
    for (size_t i = 0; i < new_path.path.size(); ++i) {
        std::cout << g[new_path.path[i]].label;
        if (i < new_path.path.size() - 1)
            std::cout << " -> ";
    }
    std::cout << " (dist: " << new_path.distance << ")" << std::endl;
    print_separator();

    // 9. REMOVE EDGE
    std::cout << "9. Removing edge A-E..." << std::endl;
    std::cout << "   Edges before: " << g.edge_count() << std::endl;

    graphix::vertex::remove_edge(e_ae, g);

    std::cout << "   Edges after: " << g.edge_count() << std::endl;
    std::cout << "   Edge A-E exists: " << (g.has_edge(a, e) ? "yes" : "no") << std::endl;
    std::cout << "   Degree of E: " << g.degree(e) << std::endl;
    print_separator();

    // 10. REMOVE VERTEX
    std::cout << "10. Removing vertex E..." << std::endl;
    std::cout << "    Vertices before: " << g.vertex_count() << std::endl;
    std::cout << "    Edges before: " << g.edge_count() << std::endl;

    graphix::vertex::remove_vertex(e, g);

    std::cout << "    Vertices after: " << g.vertex_count() << std::endl;
    std::cout << "    Edges after: " << g.edge_count() << std::endl;
    std::cout << "    Vertex E exists: " << (g.has_vertex(e) ? "yes" : "no") << std::endl;
    print_separator();

    // 11. REMAINING GRAPH STRUCTURE
    std::cout << "11. Final graph structure (square A-B-C-D):" << std::endl;
    std::cout << "    Vertices: ";
    for (auto v : g.vertices()) {
        std::cout << g[v].label << " ";
    }
    std::cout << std::endl;

    std::cout << "    Edges:" << std::endl;
    for (const auto &edge : g.edges()) {
        auto src = g.source(edge.id);
        auto tgt = g.target(edge.id);
        std::cout << "      " << g[src].label << " <-> " << g[tgt].label << std::endl;
    }
    print_separator();

    // 12. CLEAR GRAPH
    std::cout << "12. Clearing entire graph..." << std::endl;
    graphix::vertex::clear_graph(g);

    std::cout << "    Vertices: " << g.vertex_count() << std::endl;
    std::cout << "    Edges: " << g.edge_count() << std::endl;
    print_separator();

    std::cout << "=== All features demonstrated successfully! ===" << std::endl;

    return 0;
}
