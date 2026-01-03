#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graphix {
    namespace vertex {

        // Forward declarations (algorithms work with any Graph type)
        template <typename VertexProperty> class Graph;

        namespace algorithms {

            // Result of A* search
            template <typename VertexProperty> struct AStarResult {
                bool found;      // Whether a path was found
                double distance; // Total distance/cost (g-score)
                std::vector<typename Graph<VertexProperty>::VertexId>
                    path;              // Sequence of vertices from source to target
                size_t nodes_explored; // Number of nodes explored (for performance comparison)

                AStarResult() : found(false), distance(std::numeric_limits<double>::infinity()), nodes_explored(0) {}
            };

            // Priority queue element for A* algorithm
            template <typename VertexProperty> struct AStarPQElement {
                typename Graph<VertexProperty>::VertexId vertex;
                double f_score; // f(n) = g(n) + h(n)

                bool operator>(const AStarPQElement &other) const { return f_score > other.f_score; }
            };

            // A* Search Algorithm (template implementation)
            // Finds the shortest path using a heuristic function to guide the search
            // The heuristic must be admissible (never overestimates) for optimality
            //
            // Parameters:
            //   graph - The graph to search
            //   source - Starting vertex
            //   target - Goal vertex
            //   heuristic - Function that estimates distance from any vertex to target
            //               Signature: double heuristic(VertexId vertex, VertexId target)
            //
            // Returns: AStarResult with path, distance, and exploration statistics
            template <typename VertexProperty, typename HeuristicFunc>
            AStarResult<VertexProperty>
            astar(const Graph<VertexProperty> &graph, typename Graph<VertexProperty>::VertexId source,
                  typename Graph<VertexProperty>::VertexId target, HeuristicFunc heuristic) {
                using VertexId = typename Graph<VertexProperty>::VertexId;

                AStarResult<VertexProperty> result;

                // Check if source and target exist
                if (!graph.has_vertex(source) || !graph.has_vertex(target)) {
                    return result; // Not found
                }

                // Special case: source == target
                if (source == target) {
                    result.found = true;
                    result.distance = 0.0;
                    result.path.push_back(source);
                    result.nodes_explored = 1;
                    return result;
                }

                // g_score: cost from source to vertex
                // f_score: g_score + heuristic (estimated total cost)
                std::unordered_map<VertexId, double> g_score;
                std::unordered_map<VertexId, double> f_score;
                std::unordered_map<VertexId, VertexId> predecessors;
                std::unordered_set<VertexId> closed_set; // Already evaluated

                // Priority queue (min-heap based on f_score)
                std::priority_queue<AStarPQElement<VertexProperty>, std::vector<AStarPQElement<VertexProperty>>,
                                    std::greater<AStarPQElement<VertexProperty>>>
                    open_set;

                // Initialize all g_scores to infinity
                for (auto v : graph.vertices()) {
                    g_score[v] = std::numeric_limits<double>::infinity();
                    f_score[v] = std::numeric_limits<double>::infinity();
                }

                // Start from source
                g_score[source] = 0.0;
                f_score[source] = heuristic(source, target);
                open_set.push({source, f_score[source]});

                while (!open_set.empty()) {
                    auto current = open_set.top().vertex;
                    open_set.pop();

                    // Skip if already evaluated
                    if (closed_set.count(current)) {
                        continue;
                    }

                    closed_set.insert(current);
                    result.nodes_explored++;

                    // Found the target
                    if (current == target) {
                        result.found = true;
                        result.distance = g_score[target];

                        // Reconstruct path
                        std::vector<VertexId> path;
                        VertexId node = target;
                        while (node != source) {
                            path.push_back(node);
                            node = predecessors[node];
                        }
                        path.push_back(source);

                        // Reverse path to get source -> target order
                        std::reverse(path.begin(), path.end());
                        result.path = path;

                        return result;
                    }

                    // Explore neighbors
                    for (auto neighbor : graph.neighbors(current)) {
                        if (closed_set.count(neighbor)) {
                            continue;
                        }

                        // Find edge weight
                        double edge_weight = 0.0;
                        for (const auto &edge : graph.edges()) {
                            if ((edge.source == current && edge.target == neighbor) ||
                                (edge.source == neighbor && edge.target == current)) {
                                edge_weight = edge.weight;
                                break;
                            }
                        }

                        double tentative_g_score = g_score[current] + edge_weight;

                        // This path to neighbor is better than previous one
                        if (tentative_g_score < g_score[neighbor]) {
                            predecessors[neighbor] = current;
                            g_score[neighbor] = tentative_g_score;
                            f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, target);
                            open_set.push({neighbor, f_score[neighbor]});
                        }
                    }
                }

                // No path found
                return result;
            }

            // A* with zero heuristic (equivalent to Dijkstra's algorithm)
            template <typename VertexProperty>
            AStarResult<VertexProperty> astar_dijkstra(const Graph<VertexProperty> &graph,
                                                       typename Graph<VertexProperty>::VertexId source,
                                                       typename Graph<VertexProperty>::VertexId target) {
                using VertexId = typename Graph<VertexProperty>::VertexId;
                return astar(graph, source, target, [](VertexId, VertexId) { return 0.0; });
            }

            // Common heuristic: Manhattan distance (for grid graphs)
            // Assumes vertices are indexed as (row * width + col)
            inline double manhattan_distance(size_t v1, size_t v2, size_t grid_width) {
                size_t r1 = v1 / grid_width;
                size_t c1 = v1 % grid_width;
                size_t r2 = v2 / grid_width;
                size_t c2 = v2 % grid_width;
                return std::abs(static_cast<int>(r1) - static_cast<int>(r2)) +
                       std::abs(static_cast<int>(c1) - static_cast<int>(c2));
            }

            // Common heuristic: Euclidean distance (for 2D coordinate graphs)
            // Requires vertex properties with x, y fields
            template <typename Point> inline double euclidean_distance(const Point &p1, const Point &p2) {
                double dx = p1.x - p2.x;
                double dy = p1.y - p2.y;
                return std::sqrt(dx * dx + dy * dy);
            }

        } // namespace algorithms
    } // namespace vertex
} // namespace graphix
