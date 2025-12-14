#include "graphix/vertex/algorithms/bellman_ford.hpp"
#include "graphix/vertex/graph.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

using namespace graphix::vertex;

void print_distances(const BellmanFordResult &result, const Graph<std::string> &g, size_t source) {
    std::cout << "\n  Shortest distances from " << g[source] << ":\n";
    for (const auto &[vertex, distance] : result.distances) {
        std::cout << "    " << g[vertex] << ": ";
        if (std::isinf(distance)) {
            std::cout << "∞ (unreachable)\n";
        } else {
            std::cout << std::fixed << std::setprecision(1) << distance << "\n";
        }
    }
}

void print_path(const std::vector<size_t> &path, const Graph<std::string> &g) {
    if (path.empty()) {
        std::cout << "  No path found\n";
        return;
    }

    std::cout << "  Path: ";
    for (size_t i = 0; i < path.size(); i++) {
        std::cout << g[path[i]];
        if (i < path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << "\n";
}

int main() {
    std::cout << "=== Bellman-Ford Algorithm Examples ===\n\n";

    // Example 1: Basic shortest paths with positive weights
    std::cout << "Example 1: Basic Shortest Paths\n";
    std::cout << "--------------------------------\n";
    {
        Graph<std::string> g;

        auto a = g.add_vertex("A");
        auto b = g.add_vertex("B");
        auto c = g.add_vertex("C");
        auto d = g.add_vertex("D");

        g.add_edge(a, b, 4.0, EdgeType::Directed);
        g.add_edge(a, c, 2.0, EdgeType::Directed);
        g.add_edge(b, d, 3.0, EdgeType::Directed);
        g.add_edge(c, b, 1.0, EdgeType::Directed);
        g.add_edge(c, d, 7.0, EdgeType::Directed);

        std::cout << "Edges: A->B(4), A->C(2), B->D(3), C->B(1), C->D(7)\n";

        auto result = bellman_ford(g, a);
        print_distances(result, g, a);

        std::cout << "\n  Shortest path from A to D:\n";
        auto path = get_shortest_path(g, a, d);
        print_path(path, g);
        std::cout << "  (A->C->B->D with cost: " << result.distances[d] << ")\n";
    }

    std::cout << "\n";

    // Example 2: Negative edge weights (where Dijkstra fails!)
    std::cout << "Example 2: Negative Edge Weights\n";
    std::cout << "---------------------------------\n";
    {
        Graph<std::string> g;

        auto a = g.add_vertex("A");
        auto b = g.add_vertex("B");
        auto c = g.add_vertex("C");
        auto d = g.add_vertex("D");

        g.add_edge(a, b, 5.0, EdgeType::Directed);
        g.add_edge(a, c, 2.0, EdgeType::Directed);
        g.add_edge(b, d, 1.0, EdgeType::Directed);
        g.add_edge(c, b, -4.0, EdgeType::Directed); // Negative weight!
        g.add_edge(c, d, 3.0, EdgeType::Directed);

        std::cout << "Edges: A->B(5), A->C(2), B->D(1), C->B(-4), C->D(3)\n";
        std::cout << "Note: C->B has NEGATIVE weight (-4)\n";

        auto result = bellman_ford(g, a);
        print_distances(result, g, a);

        std::cout << "\n  Better path to B via negative edge:\n";
        auto path_b = get_shortest_path(g, a, b);
        print_path(path_b, g);
        std::cout << "  Cost via C: " << result.distances[b] << " (better than direct 5.0)\n";

        std::cout << "\n  Bellman-Ford handles this correctly!\n";
        std::cout << "  Dijkstra would fail because it doesn't support negative weights.\n";
    }

    std::cout << "\n";

    // Example 3: Negative cycle detection
    std::cout << "Example 3: Negative Cycle Detection\n";
    std::cout << "------------------------------------\n";
    {
        Graph<std::string> g;

        auto a = g.add_vertex("A");
        auto b = g.add_vertex("B");
        auto c = g.add_vertex("C");

        g.add_edge(a, b, 1.0, EdgeType::Directed);
        g.add_edge(b, c, -3.0, EdgeType::Directed);
        g.add_edge(c, a, 1.0, EdgeType::Directed); // Cycle: 1 + (-3) + 1 = -1

        std::cout << "Edges: A->B(1), B->C(-3), C->A(1)\n";
        std::cout << "Cycle sum: 1 + (-3) + 1 = -1 (negative!)\n";

        auto result = bellman_ford(g, a);

        if (result.has_negative_cycle) {
            std::cout << "\n  ❌ NEGATIVE CYCLE DETECTED!\n";
            std::cout << "  Cycle vertices: ";
            for (size_t i = 0; i < result.negative_cycle.size(); i++) {
                std::cout << g[result.negative_cycle[i]];
                if (i < result.negative_cycle.size() - 1) {
                    std::cout << " -> ";
                }
            }
            std::cout << "\n";
            std::cout << "\n  No shortest path exists (can keep reducing by going around cycle)\n";
        }
    }

    std::cout << "\n";

    // Example 4: Currency arbitrage detection
    std::cout << "Example 4: Currency Arbitrage Detection\n";
    std::cout << "----------------------------------------\n";
    {
        Graph<std::string> g;

        auto usd = g.add_vertex("USD");
        auto eur = g.add_vertex("EUR");
        auto gbp = g.add_vertex("GBP");
        auto jpy = g.add_vertex("JPY");

        // Exchange rates (using negative log to convert to distances)
        // If we can find negative cycle, we have arbitrage opportunity
        g.add_edge(usd, eur, -std::log(0.85), EdgeType::Directed);   // 1 USD = 0.85 EUR
        g.add_edge(eur, gbp, -std::log(0.90), EdgeType::Directed);   // 1 EUR = 0.90 GBP
        g.add_edge(gbp, jpy, -std::log(140.0), EdgeType::Directed);  // 1 GBP = 140 JPY
        g.add_edge(jpy, usd, -std::log(0.0095), EdgeType::Directed); // 1 JPY = 0.0095 USD

        // Manipulated rate to create arbitrage
        g.add_edge(eur, usd, -std::log(1.2), EdgeType::Directed); // 1 EUR = 1.2 USD (too good!)

        std::cout << "Exchange rates with one manipulated rate...\n";

        auto result = bellman_ford(g, usd);

        if (result.has_negative_cycle) {
            std::cout << "\n  💰 ARBITRAGE OPPORTUNITY DETECTED!\n";
            std::cout << "  Can profit by cycling through currencies:\n  ";
            for (size_t i = 0; i < result.negative_cycle.size(); i++) {
                std::cout << g[result.negative_cycle[i]];
                if (i < result.negative_cycle.size() - 1) {
                    std::cout << " -> ";
                }
            }
            std::cout << "\n";
        }
    }

    std::cout << "\n";

    // Example 5: Network routing with congestion penalties
    std::cout << "Example 5: Network Routing with Penalties\n";
    std::cout << "------------------------------------------\n";
    {
        Graph<std::string> g;

        auto src = g.add_vertex("Source");
        auto r1 = g.add_vertex("Router1");
        auto r2 = g.add_vertex("Router2");
        auto r3 = g.add_vertex("Router3");
        auto dst = g.add_vertex("Destination");

        // Positive weights = latency
        g.add_edge(src, r1, 10.0, EdgeType::Directed);
        g.add_edge(src, r2, 15.0, EdgeType::Directed);
        g.add_edge(r1, r3, 12.0, EdgeType::Directed);
        g.add_edge(r2, r3, 8.0, EdgeType::Directed);

        // Negative weight = cache hit bonus (reduces effective cost)
        g.add_edge(r1, dst, -3.0, EdgeType::Directed); // Cached route!
        g.add_edge(r3, dst, 5.0, EdgeType::Directed);

        std::cout << "Network with cache bonus (negative weight)\n";
        std::cout << "Source->Router1(10), Router1->Dest(-3, cached!)\n";

        auto result = bellman_ford(g, src);

        if (!result.has_negative_cycle) {
            std::cout << "\n  Optimal route:\n";
            auto path = get_shortest_path(g, src, dst);
            print_path(path, g);
            std::cout << "  Total cost: " << result.distances[dst] << " ms\n";
            std::cout << "  (Cache bonus made this route faster!)\n";
        }
    }

    std::cout << "\n";

    // Example 6: Timeline with time travel (negative time edges)
    std::cout << "Example 6: Temporal Network\n";
    std::cout << "----------------------------\n";
    {
        Graph<std::string> g;

        auto t0 = g.add_vertex("T0 (9:00 AM)");
        auto t1 = g.add_vertex("T1 (10:00 AM)");
        auto t2 = g.add_vertex("T2 (11:00 AM)");
        auto t3 = g.add_vertex("T3 (12:00 PM)");

        // Normal time flow
        g.add_edge(t0, t1, 60.0, EdgeType::Directed); // 60 minutes
        g.add_edge(t1, t2, 60.0, EdgeType::Directed);

        // "Fast travel" opportunity (e.g., shortcut, teleporter)
        g.add_edge(t1, t3, 30.0, EdgeType::Directed); // Skip ahead

        // "Rewind" mechanism (negative time cost)
        g.add_edge(t2, t1, -30.0, EdgeType::Directed); // Go back 30 min

        std::cout << "Temporal network with time rewind capability\n";

        auto result = bellman_ford(g, t0);

        if (result.has_negative_cycle) {
            std::cout << "\n  ⚠️  Time paradox detected (negative cycle)!\n";
        } else {
            print_distances(result, g, t0);
        }
    }

    std::cout << "\n";

    // Example 7: Comparing with simple case
    std::cout << "Example 7: When Bellman-Ford equals Dijkstra\n";
    std::cout << "---------------------------------------------\n";
    {
        Graph<std::string> g;

        auto a = g.add_vertex("A");
        auto b = g.add_vertex("B");
        auto c = g.add_vertex("C");
        auto d = g.add_vertex("D");

        // All positive weights
        g.add_edge(a, b, 2.0, EdgeType::Directed);
        g.add_edge(a, c, 4.0, EdgeType::Directed);
        g.add_edge(b, c, 1.0, EdgeType::Directed);
        g.add_edge(b, d, 7.0, EdgeType::Directed);
        g.add_edge(c, d, 3.0, EdgeType::Directed);

        std::cout << "All positive weights (like Dijkstra's requirements)\n";

        auto result = bellman_ford(g, a);
        print_distances(result, g, a);

        std::cout << "\n  Both Bellman-Ford and Dijkstra give same result.\n";
        std::cout << "  But Bellman-Ford is O(VE) vs Dijkstra's O(E log V)\n";
        std::cout << "  Use Dijkstra for all-positive weights (faster)!\n";
    }

    std::cout << "\n";

    // Example 8: Game score optimization
    std::cout << "Example 8: Game Score Optimization\n";
    std::cout << "-----------------------------------\n";
    {
        Graph<std::string> g;

        auto start = g.add_vertex("Start");
        auto room1 = g.add_vertex("Room 1");
        auto room2 = g.add_vertex("Room 2");
        auto treasure = g.add_vertex("Treasure");
        auto exit = g.add_vertex("Exit");

        // Positive = energy cost, Negative = energy gain
        g.add_edge(start, room1, 10.0, EdgeType::Directed); // costs energy
        g.add_edge(start, room2, 15.0, EdgeType::Directed);
        g.add_edge(room1, treasure, -20.0, EdgeType::Directed); // health potion!
        g.add_edge(room2, treasure, 5.0, EdgeType::Directed);
        g.add_edge(treasure, exit, 8.0, EdgeType::Directed);

        std::cout << "Game map with energy costs/gains\n";
        std::cout << "Room 1 -> Treasure gives HEALTH POTION (-20 cost = gain!)\n";

        auto result = bellman_ford(g, start);

        if (!result.has_negative_cycle) {
            std::cout << "\n  Best route to exit:\n";
            auto path = get_shortest_path(g, start, exit);
            print_path(path, g);
            std::cout << "  Net energy: " << result.distances[exit] << "\n";
            std::cout << "  (Negative means you GAIN energy overall!)\n";
        }
    }

    std::cout << "\n";
    std::cout << "=== Summary ===\n";
    std::cout << "Bellman-Ford Algorithm:\n";
    std::cout << "  ✓ Handles negative edge weights\n";
    std::cout << "  ✓ Detects negative cycles\n";
    std::cout << "  ✓ More general than Dijkstra\n";
    std::cout << "  ✓ Time complexity: O(V·E)\n";
    std::cout << "  ✓ Use when: negative weights present or cycle detection needed\n";

    return 0;
}
