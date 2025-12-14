#include "graphix/vertex/algorithms/dijkstra.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>
#include <string>

// Dijkstra shortest path example: Find shortest paths in a city road network

struct City {
    std::string name;

    City(const std::string &n = "") : name(n) {}
};

int main() {
    std::cout << "=== Dijkstra Shortest Path Example ===" << std::endl;
    std::cout << std::endl;

    // Create a graph with city names as vertex properties
    graphix::vertex::Graph<City> g;

    // Add cities
    std::cout << "Building road network..." << std::endl;
    auto seattle = g.add_vertex(City("Seattle"));
    auto portland = g.add_vertex(City("Portland"));
    auto sf = g.add_vertex(City("San Francisco"));
    auto la = g.add_vertex(City("Los Angeles"));
    auto vegas = g.add_vertex(City("Las Vegas"));
    auto phoenix = g.add_vertex(City("Phoenix"));

    std::cout << "  Created " << g.vertex_count() << " cities" << std::endl;
    std::cout << std::endl;

    // Add roads with distances (in hundreds of miles, roughly)
    std::cout << "Adding roads with distances..." << std::endl;
    g.add_edge(seattle, portland, 1.7); // 170 miles
    g.add_edge(portland, sf, 6.4);      // 640 miles
    g.add_edge(sf, la, 3.8);            // 380 miles
    g.add_edge(la, vegas, 2.7);         // 270 miles
    g.add_edge(la, phoenix, 3.7);       // 370 miles
    g.add_edge(vegas, phoenix, 2.9);    // 290 miles
    g.add_edge(seattle, sf, 8.1);       // 810 miles (scenic route)
    g.add_edge(portland, la, 9.6);      // 960 miles (long route)

    std::cout << "  Created " << g.edge_count() << " roads" << std::endl;
    std::cout << std::endl;

    // Find shortest path from Seattle to Phoenix
    std::cout << "Finding shortest path from Seattle to Phoenix..." << std::endl;
    auto result = graphix::vertex::algorithms::dijkstra(g, seattle, phoenix);

    if (result.found) {
        std::cout << "  Distance: " << result.distance << " (×100 miles)" << std::endl;
        std::cout << "  Route: ";
        for (size_t i = 0; i < result.path.size(); ++i) {
            std::cout << g[result.path[i]].name;
            if (i < result.path.size() - 1) {
                std::cout << " -> ";
            }
        }
        std::cout << std::endl;
    } else {
        std::cout << "  No path found!" << std::endl;
    }
    std::cout << std::endl;

    // Find all shortest paths from Seattle
    std::cout << "Shortest distances from Seattle to all cities:" << std::endl;
    for (auto v : g.vertices()) {
        if (v == seattle)
            continue;

        auto r = graphix::vertex::algorithms::dijkstra(g, seattle, v);
        if (r.found) {
            std::cout << "  To " << g[v].name << ": " << r.distance << " (×100 mi)";
            std::cout << " via " << r.path.size() << " cities" << std::endl;
        } else {
            std::cout << "  To " << g[v].name << ": unreachable" << std::endl;
        }
    }
    std::cout << std::endl;

    // Demonstrate disconnected component
    std::cout << "Adding disconnected city (Honolulu)..." << std::endl;
    auto honolulu = g.add_vertex(City("Honolulu"));

    auto island_result = graphix::vertex::algorithms::dijkstra(g, seattle, honolulu);
    if (!island_result.found) {
        std::cout << "  Seattle to Honolulu: No path (disconnected component)" << std::endl;
        std::cout << "  Distance: " << island_result.distance << " (infinity)" << std::endl;
    }
    std::cout << std::endl;

    // Compare different routes
    std::cout << "Route comparison (Seattle to Los Angeles):" << std::endl;

    auto direct_sf = graphix::vertex::algorithms::dijkstra(g, seattle, la);
    std::cout << "  Best route distance: " << direct_sf.distance << std::endl;

    // Show the actual route taken
    std::cout << "  Best route path: ";
    for (size_t i = 0; i < direct_sf.path.size(); ++i) {
        std::cout << g[direct_sf.path[i]].name;
        if (i < direct_sf.path.size() - 1)
            std::cout << " -> ";
    }
    std::cout << std::endl;

    return 0;
}
