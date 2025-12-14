#pragma once

#include <algorithm>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graphix {
    namespace vertex {

        // Forward declarations (algorithms work with any Graph type)
        template <typename VertexProperty> class Graph;

        namespace algorithms {

            // Result of shortest path computation
            template <typename VertexProperty> struct ShortestPathResult {
                bool found;      // Whether a path was found
                double distance; // Total distance/cost
                std::vector<typename Graph<VertexProperty>::VertexId>
                    path; // Sequence of vertices from source to target

                ShortestPathResult() : found(false), distance(std::numeric_limits<double>::infinity()) {}
            };

            // Priority queue element for Dijkstra's algorithm
            template <typename VertexProperty> struct PQElement {
                typename Graph<VertexProperty>::VertexId vertex;
                double distance;

                bool operator>(const PQElement &other) const { return distance > other.distance; }
            };

            // Dijkstra's shortest path algorithm (template implementation)
            // Finds the shortest path between source and target vertices using edge weights
            template <typename VertexProperty>
            ShortestPathResult<VertexProperty> dijkstra(const Graph<VertexProperty> &graph,
                                                        typename Graph<VertexProperty>::VertexId source,
                                                        typename Graph<VertexProperty>::VertexId target) {
                using VertexId = typename Graph<VertexProperty>::VertexId;

                ShortestPathResult<VertexProperty> result;

                // Check if source and target exist
                if (!graph.has_vertex(source) || !graph.has_vertex(target)) {
                    return result; // Not found
                }

                // Special case: source == target
                if (source == target) {
                    result.found = true;
                    result.distance = 0.0;
                    result.path.push_back(source);
                    return result;
                }

                // Initialize distances and predecessors
                std::unordered_map<VertexId, double> distances;
                std::unordered_map<VertexId, VertexId> predecessors;
                std::unordered_set<VertexId> visited;

                // Priority queue (min-heap)
                std::priority_queue<PQElement<VertexProperty>, std::vector<PQElement<VertexProperty>>,
                                    std::greater<PQElement<VertexProperty>>>
                    pq;

                // Initialize all distances to infinity
                for (auto v : graph.vertices()) {
                    distances[v] = std::numeric_limits<double>::infinity();
                }

                // Start from source
                distances[source] = 0.0;
                pq.push({source, 0.0});

                while (!pq.empty()) {
                    auto current = pq.top().vertex;
                    pq.pop();

                    // Skip if already visited
                    if (visited.count(current)) {
                        continue;
                    }

                    visited.insert(current);

                    // Found the target
                    if (current == target) {
                        break;
                    }

                    // Relax edges to neighbors
                    for (auto neighbor : graph.neighbors(current)) {
                        if (visited.count(neighbor)) {
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

                        double new_distance = distances[current] + edge_weight;

                        if (new_distance < distances[neighbor]) {
                            distances[neighbor] = new_distance;
                            predecessors[neighbor] = current;
                            pq.push({neighbor, new_distance});
                        }
                    }
                }

                // Check if target was reached
                if (distances[target] == std::numeric_limits<double>::infinity()) {
                    return result; // No path found
                }

                // Reconstruct path
                result.found = true;
                result.distance = distances[target];

                std::vector<VertexId> path;
                VertexId current = target;
                while (current != source) {
                    path.push_back(current);
                    current = predecessors[current];
                }
                path.push_back(source);

                // Reverse path to get source -> target order
                std::reverse(path.begin(), path.end());
                result.path = path;

                return result;
            }

        } // namespace algorithms
    } // namespace vertex
} // namespace graphix
