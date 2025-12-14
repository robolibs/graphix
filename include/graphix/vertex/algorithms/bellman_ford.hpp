#pragma once

#include "graphix/vertex/graph.hpp"
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <vector>

namespace graphix {
    namespace vertex {

        // Bellman-Ford algorithm result
        struct BellmanFordResult {
            std::unordered_map<size_t, double> distances;    // Shortest distances from source
            std::unordered_map<size_t, size_t> predecessors; // Predecessor in shortest path
            bool has_negative_cycle;                         // True if negative cycle detected
            std::vector<size_t> negative_cycle;              // Vertices in negative cycle (if any)

            BellmanFordResult() : has_negative_cycle(false) {}
        };

        // ============================================================================
        // Bellman-Ford Algorithm
        // ============================================================================
        // Finds shortest paths from a single source vertex
        // Can handle negative edge weights
        // Detects negative cycles
        // Time complexity: O(V * E)
        // Space complexity: O(V)

        template <typename VertexProperty>
        BellmanFordResult bellman_ford(const Graph<VertexProperty> &g, size_t source) {
            BellmanFordResult result;

            auto verts = g.vertices();
            if (verts.empty()) {
                return result;
            }

            // Check if source vertex exists
            if (std::find(verts.begin(), verts.end(), source) == verts.end()) {
                return result;
            }

            // Initialize distances
            const double INF = std::numeric_limits<double>::infinity();
            for (auto v : verts) {
                result.distances[v] = INF;
            }
            result.distances[source] = 0.0;

            // Get all edges
            auto edges = g.edges();

            // Relax edges |V| - 1 times
            for (size_t i = 0; i < verts.size() - 1; i++) {
                bool updated = false;

                for (const auto &edge : edges) {
                    // Only process directed edges
                    if (edge.type != EdgeType::Directed) {
                        continue;
                    }

                    size_t u = edge.source;
                    size_t v = edge.target;
                    double weight = edge.weight;

                    if (result.distances[u] != INF && result.distances[u] + weight < result.distances[v]) {
                        result.distances[v] = result.distances[u] + weight;
                        result.predecessors[v] = u;
                        updated = true;
                    }
                }

                // Early termination if no updates in this iteration
                if (!updated) {
                    break;
                }
            }

            // Check for negative cycles
            for (const auto &edge : edges) {
                if (edge.type != EdgeType::Directed) {
                    continue;
                }

                size_t u = edge.source;
                size_t v = edge.target;
                double weight = edge.weight;

                if (result.distances[u] != INF && result.distances[u] + weight < result.distances[v]) {
                    // Negative cycle detected
                    result.has_negative_cycle = true;

                    // Trace back to find the cycle
                    std::unordered_map<size_t, bool> visited;
                    size_t current = v;

                    // Go back |V| steps to ensure we're in the cycle
                    for (size_t i = 0; i < verts.size(); i++) {
                        if (result.predecessors.find(current) != result.predecessors.end()) {
                            current = result.predecessors[current];
                        }
                    }

                    // Now trace the cycle
                    size_t cycle_start = current;
                    result.negative_cycle.push_back(current);
                    visited[current] = true;

                    current = result.predecessors[current];
                    while (current != cycle_start) {
                        result.negative_cycle.push_back(current);
                        visited[current] = true;
                        if (result.predecessors.find(current) != result.predecessors.end()) {
                            current = result.predecessors[current];
                        } else {
                            break;
                        }
                    }

                    result.negative_cycle.push_back(cycle_start);
                    std::reverse(result.negative_cycle.begin(), result.negative_cycle.end());

                    break;
                }
            }

            return result;
        }

        // Specialization for Graph<void>
        inline BellmanFordResult bellman_ford(const Graph<void> &g, size_t source) {
            BellmanFordResult result;

            auto verts = g.vertices();
            if (verts.empty()) {
                return result;
            }

            // Check if source vertex exists
            if (std::find(verts.begin(), verts.end(), source) == verts.end()) {
                return result;
            }

            // Initialize distances
            const double INF = std::numeric_limits<double>::infinity();
            for (auto v : verts) {
                result.distances[v] = INF;
            }
            result.distances[source] = 0.0;

            // Get all edges
            auto edges = g.edges();

            // Relax edges |V| - 1 times
            for (size_t i = 0; i < verts.size() - 1; i++) {
                bool updated = false;

                for (const auto &edge : edges) {
                    // Only process directed edges
                    if (edge.type != EdgeType::Directed) {
                        continue;
                    }

                    size_t u = edge.source;
                    size_t v = edge.target;
                    double weight = edge.weight;

                    if (result.distances[u] != INF && result.distances[u] + weight < result.distances[v]) {
                        result.distances[v] = result.distances[u] + weight;
                        result.predecessors[v] = u;
                        updated = true;
                    }
                }

                // Early termination if no updates in this iteration
                if (!updated) {
                    break;
                }
            }

            // Check for negative cycles
            for (const auto &edge : edges) {
                if (edge.type != EdgeType::Directed) {
                    continue;
                }

                size_t u = edge.source;
                size_t v = edge.target;
                double weight = edge.weight;

                if (result.distances[u] != INF && result.distances[u] + weight < result.distances[v]) {
                    // Negative cycle detected
                    result.has_negative_cycle = true;

                    // Trace back to find the cycle
                    std::unordered_map<size_t, bool> visited;
                    size_t current = v;

                    // Go back |V| steps to ensure we're in the cycle
                    for (size_t i = 0; i < verts.size(); i++) {
                        if (result.predecessors.find(current) != result.predecessors.end()) {
                            current = result.predecessors[current];
                        }
                    }

                    // Now trace the cycle
                    size_t cycle_start = current;
                    result.negative_cycle.push_back(current);
                    visited[current] = true;

                    current = result.predecessors[current];
                    while (current != cycle_start) {
                        result.negative_cycle.push_back(current);
                        visited[current] = true;
                        if (result.predecessors.find(current) != result.predecessors.end()) {
                            current = result.predecessors[current];
                        } else {
                            break;
                        }
                    }

                    result.negative_cycle.push_back(cycle_start);
                    std::reverse(result.negative_cycle.begin(), result.negative_cycle.end());

                    break;
                }
            }

            return result;
        }

        // ============================================================================
        // Helper Functions
        // ============================================================================

        // Get shortest path from source to target
        template <typename VertexProperty>
        std::vector<size_t> get_shortest_path(const Graph<VertexProperty> &g, size_t source, size_t target) {
            auto result = bellman_ford(g, source);

            if (result.has_negative_cycle) {
                return {}; // No valid path if negative cycle exists
            }

            if (result.distances.find(target) == result.distances.end() ||
                result.distances[target] == std::numeric_limits<double>::infinity()) {
                return {}; // No path exists
            }

            // Reconstruct path
            std::vector<size_t> path;
            size_t current = target;

            while (current != source) {
                path.push_back(current);
                if (result.predecessors.find(current) == result.predecessors.end()) {
                    return {}; // Path broken
                }
                current = result.predecessors[current];
            }

            path.push_back(source);
            std::reverse(path.begin(), path.end());
            return path;
        }

        inline std::vector<size_t> get_shortest_path(const Graph<void> &g, size_t source, size_t target) {
            auto result = bellman_ford(g, source);

            if (result.has_negative_cycle) {
                return {}; // No valid path if negative cycle exists
            }

            if (result.distances.find(target) == result.distances.end() ||
                result.distances[target] == std::numeric_limits<double>::infinity()) {
                return {}; // No path exists
            }

            // Reconstruct path
            std::vector<size_t> path;
            size_t current = target;

            while (current != source) {
                path.push_back(current);
                if (result.predecessors.find(current) == result.predecessors.end()) {
                    return {}; // Path broken
                }
                current = result.predecessors[current];
            }

            path.push_back(source);
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Check if there's a negative cycle reachable from source
        template <typename VertexProperty>
        inline bool has_negative_cycle(const Graph<VertexProperty> &g, size_t source) {
            return bellman_ford(g, source).has_negative_cycle;
        }

        inline bool has_negative_cycle(const Graph<void> &g, size_t source) {
            return bellman_ford(g, source).has_negative_cycle;
        }

        // Get distance to a specific target
        template <typename VertexProperty>
        inline double get_distance(const Graph<VertexProperty> &g, size_t source, size_t target) {
            auto result = bellman_ford(g, source);
            if (result.distances.find(target) != result.distances.end()) {
                return result.distances[target];
            }
            return std::numeric_limits<double>::infinity();
        }

        inline double get_distance(const Graph<void> &g, size_t source, size_t target) {
            auto result = bellman_ford(g, source);
            if (result.distances.find(target) != result.distances.end()) {
                return result.distances[target];
            }
            return std::numeric_limits<double>::infinity();
        }

    } // namespace vertex
} // namespace graphix
