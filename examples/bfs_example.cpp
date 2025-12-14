#include "graphix/vertex/algorithms/bfs.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>

using namespace graphix::vertex;

int main() {
    std::cout << "=== BFS (Breadth-First Search) Example ===" << std::endl;

    Graph<std::string> g;

    // Create a social network graph
    auto alice = g.add_vertex("Alice");
    auto bob = g.add_vertex("Bob");
    auto charlie = g.add_vertex("Charlie");
    auto diana = g.add_vertex("Diana");
    auto eve = g.add_vertex("Eve");
    auto frank = g.add_vertex("Frank");

    // Add friendships (edges)
    g.add_edge(alice, bob);
    g.add_edge(alice, charlie);
    g.add_edge(bob, diana);
    g.add_edge(charlie, eve);
    g.add_edge(diana, frank);
    g.add_edge(eve, frank);

    std::cout << "\nSocial Network:" << std::endl;
    std::cout << "  Alice --- Bob --- Diana" << std::endl;
    std::cout << "    |                 |" << std::endl;
    std::cout << "  Charlie --- Eve --- Frank" << std::endl;

    // Example 1: BFS from Alice to explore the network
    std::cout << "\n--- Example 1: BFS from Alice ---" << std::endl;
    auto result1 = algorithms::bfs(g, alice);

    std::cout << "Discovery order: ";
    for (size_t i = 0; i < result1.discovery_order.size(); ++i) {
        auto v = result1.discovery_order[i];
        std::cout << g[v];
        if (i < result1.discovery_order.size() - 1)
            std::cout << " -> ";
    }
    std::cout << std::endl;

    std::cout << "\nDistances from Alice:" << std::endl;
    for (const auto &[vertex, distance] : result1.distance) {
        std::cout << "  " << g[vertex] << ": " << distance << " steps" << std::endl;
    }

    // Example 2: Find shortest path from Alice to Frank
    std::cout << "\n--- Example 2: Shortest path from Alice to Frank ---" << std::endl;
    auto result2 = algorithms::bfs(g, alice, frank);

    if (result2.target_found) {
        auto path = algorithms::reconstruct_path(result2, alice, frank);
        std::cout << "Path (" << path.size() - 1 << " steps): ";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << g[path[i]];
            if (i < path.size() - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;
        std::cout << "Distance: " << result2.distance[frank] << " steps" << std::endl;
    } else {
        std::cout << "No path found!" << std::endl;
    }

    // Example 3: Check reachability
    std::cout << "\n--- Example 3: Checking reachability ---" << std::endl;
    auto result3 = algorithms::bfs(g, alice);

    std::cout << "People reachable from Alice: ";
    for (const auto &[vertex, _] : result3.distance) {
        if (vertex != alice) {
            std::cout << g[vertex] << " ";
        }
    }
    std::cout << std::endl;

    return 0;
}
