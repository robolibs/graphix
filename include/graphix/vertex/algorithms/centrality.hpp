#pragma once

#include "graphix/vertex/graph.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graphix {
    namespace vertex {
        namespace algorithms {

            // ========================================================================
            // Degree Centrality - Simplest centrality measure
            // ========================================================================

            // Compute degree centrality for a single vertex
            // Returns normalized degree (degree / (n-1)) for directed graphs
            // or (degree / (2*(n-1))) for undirected graphs
            template <typename VertexProperty>
            inline double degree_centrality(const graphix::vertex::Graph<VertexProperty> &graph,
                                            typename graphix::vertex::Graph<VertexProperty>::VertexId v) {
                if (!graph.has_vertex(v)) {
                    throw std::invalid_argument("Vertex does not exist in graph");
                }

                size_t n = graph.vertex_count();
                if (n <= 1) {
                    return 0.0;
                }

                // Check if graph has any directed edges
                bool has_directed = false;
                for (const auto &edge_desc : graph.edges()) {
                    if (edge_desc.type == EdgeType::Directed) {
                        has_directed = true;
                        break;
                    }
                }

                double degree_val;
                if (has_directed) {
                    // For directed graphs: count both in-degree and out-degree
                    size_t out_degree = graph.degree(v); // degree() returns out-degree
                    size_t in_degree = 0;
                    for (const auto &edge_desc : graph.edges()) {
                        if (edge_desc.target == v && edge_desc.type == EdgeType::Directed) {
                            in_degree++;
                        }
                    }
                    degree_val = static_cast<double>(in_degree + out_degree);
                } else {
                    // For undirected graphs: degree() counts each edge twice (once from each end)
                    degree_val = static_cast<double>(graph.degree(v));
                }

                double max_possible = static_cast<double>(n - 1);

                // For undirected graphs, each edge is counted twice in adjacency list
                if (!has_directed) {
                    return degree_val / (2.0 * max_possible);
                }

                return degree_val / max_possible;
            }

            // Compute degree centrality for all vertices
            // Returns map of vertex -> centrality score
            template <typename VertexProperty>
            inline std::unordered_map<typename graphix::vertex::Graph<VertexProperty>::VertexId, double>
            degree_centrality_all(const graphix::vertex::Graph<VertexProperty> &graph) {

                using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;
                std::unordered_map<VertexId, double> centrality;

                size_t n = graph.vertex_count();
                if (n <= 1) {
                    for (auto v : graph.vertices()) {
                        centrality[v] = 0.0;
                    }
                    return centrality;
                }

                // Check if graph has any directed edges
                bool has_directed = false;
                for (const auto &edge_desc : graph.edges()) {
                    if (edge_desc.type == EdgeType::Directed) {
                        has_directed = true;
                        break;
                    }
                }

                double max_possible = static_cast<double>(n - 1);
                double divisor = has_directed ? max_possible : (2.0 * max_possible);

                if (has_directed) {
                    // For directed graphs: compute in-degree for all vertices first
                    std::unordered_map<VertexId, size_t> in_degree;
                    for (auto v : graph.vertices()) {
                        in_degree[v] = 0;
                    }
                    for (const auto &edge_desc : graph.edges()) {
                        if (edge_desc.type == EdgeType::Directed) {
                            in_degree[edge_desc.target]++;
                        }
                    }

                    // Now compute centrality using in + out degree
                    for (auto v : graph.vertices()) {
                        size_t out_deg = graph.degree(v);
                        double total_degree = static_cast<double>(in_degree[v] + out_deg);
                        centrality[v] = total_degree / divisor;
                    }
                } else {
                    // For undirected graphs: degree() already counts correctly
                    for (auto v : graph.vertices()) {
                        double degree = static_cast<double>(graph.degree(v));
                        centrality[v] = degree / divisor;
                    }
                }

                return centrality;
            }

            // ========================================================================
            // Closeness Centrality - Based on shortest path distances
            // ========================================================================

            // Compute closeness centrality for a single vertex
            // Closeness = (n-1) / sum of shortest distances to all other vertices
            // Returns 0 if vertex cannot reach all others (disconnected graph)
            template <typename VertexProperty>
            inline double closeness_centrality(const graphix::vertex::Graph<VertexProperty> &graph,
                                               typename graphix::vertex::Graph<VertexProperty>::VertexId source) {

                using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

                if (!graph.has_vertex(source)) {
                    throw std::invalid_argument("Vertex does not exist in graph");
                }

                size_t n = graph.vertex_count();
                if (n <= 1) {
                    return 0.0;
                }

                // Run BFS to compute shortest distances
                std::unordered_map<VertexId, double> distance;
                std::unordered_set<VertexId> visited;
                std::queue<VertexId> queue;

                queue.push(source);
                visited.insert(source);
                distance[source] = 0.0;

                while (!queue.empty()) {
                    auto current = queue.front();
                    queue.pop();

                    for (auto edge_id : graph.out_edges(current)) {
                        VertexId src = graph.source(edge_id);
                        VertexId tgt = graph.target(edge_id);
                        VertexId neighbor = (src == current) ? tgt : src;

                        if (!visited.count(neighbor)) {
                            visited.insert(neighbor);
                            distance[neighbor] = distance[current] + 1.0; // Hop count, not weight
                            queue.push(neighbor);
                        }
                    }
                }

                // If cannot reach all vertices, return 0 (or could use Wasserman-Faust normalization)
                if (visited.size() != n) {
                    return 0.0;
                }

                // Sum all distances
                double total_distance = 0.0;
                for (const auto &[v, dist] : distance) {
                    if (v != source) {
                        total_distance += dist;
                    }
                }

                if (total_distance == 0.0) {
                    return 0.0;
                }

                // Closeness = (n-1) / sum_of_distances
                return static_cast<double>(n - 1) / total_distance;
            }

            // Compute closeness centrality for all vertices
            template <typename VertexProperty>
            inline std::unordered_map<typename graphix::vertex::Graph<VertexProperty>::VertexId, double>
            closeness_centrality_all(const graphix::vertex::Graph<VertexProperty> &graph) {

                using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;
                std::unordered_map<VertexId, double> centrality;

                for (auto v : graph.vertices()) {
                    centrality[v] = closeness_centrality(graph, v);
                }

                return centrality;
            }

            // ========================================================================
            // Betweenness Centrality - Based on shortest paths through a vertex
            // ========================================================================

            // Helper: Single-source shortest paths with path counting (for betweenness)
            template <typename VertexProperty>
            inline void betweenness_single_source(
                const graphix::vertex::Graph<VertexProperty> &graph,
                typename graphix::vertex::Graph<VertexProperty>::VertexId source,
                std::unordered_map<typename graphix::vertex::Graph<VertexProperty>::VertexId, double> &betweenness) {

                using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

                // Brandes' algorithm for betweenness centrality
                std::unordered_map<VertexId, std::vector<VertexId>> predecessors;
                std::unordered_map<VertexId, int> sigma; // number of shortest paths
                std::unordered_map<VertexId, double> distance;
                std::unordered_map<VertexId, double> delta; // dependency

                std::queue<VertexId> queue;
                std::vector<VertexId> stack; // vertices in order of non-increasing distance

                // Initialize
                for (auto v : graph.vertices()) {
                    predecessors[v] = std::vector<VertexId>();
                    sigma[v] = 0;
                    distance[v] = std::numeric_limits<double>::infinity();
                    delta[v] = 0.0;
                }

                sigma[source] = 1;
                distance[source] = 0.0;
                queue.push(source);

                // BFS to find shortest paths
                while (!queue.empty()) {
                    auto v = queue.front();
                    queue.pop();
                    stack.push_back(v);

                    for (auto edge_id : graph.out_edges(v)) {
                        VertexId src = graph.source(edge_id);
                        VertexId tgt = graph.target(edge_id);
                        VertexId w = (src == v) ? tgt : src;

                        // First time we see w?
                        if (distance[w] == std::numeric_limits<double>::infinity()) {
                            distance[w] = distance[v] + 1.0; // Hop count, not weight
                            queue.push(w);
                        }

                        // Shortest path to w via v?
                        if (std::abs(distance[w] - (distance[v] + 1.0)) < 1e-9) {
                            sigma[w] += sigma[v];
                            predecessors[w].push_back(v);
                        }
                    }
                }

                // Back-propagation to accumulate dependencies
                while (!stack.empty()) {
                    auto w = stack.back();
                    stack.pop_back();

                    for (auto v : predecessors[w]) {
                        double contribution =
                            (static_cast<double>(sigma[v]) / static_cast<double>(sigma[w])) * (1.0 + delta[w]);
                        delta[v] += contribution;
                    }

                    if (w != source) {
                        betweenness[w] += delta[w];
                    }
                }
            }

            // Compute betweenness centrality for all vertices
            // Betweenness measures how often a vertex appears on shortest paths between other vertices
            template <typename VertexProperty>
            inline std::unordered_map<typename graphix::vertex::Graph<VertexProperty>::VertexId, double>
            betweenness_centrality(const graphix::vertex::Graph<VertexProperty> &graph) {

                using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

                std::unordered_map<VertexId, double> betweenness;

                // Initialize all to 0
                for (auto v : graph.vertices()) {
                    betweenness[v] = 0.0;
                }

                // Run from each vertex as source
                for (auto s : graph.vertices()) {
                    betweenness_single_source(graph, s, betweenness);
                }

                // Normalize
                size_t n = graph.vertex_count();
                if (n <= 2) {
                    // No normalization needed for graphs with <= 2 vertices
                    return betweenness;
                }

                // Check if graph is directed
                bool has_directed = false;
                for (const auto &edge_desc : graph.edges()) {
                    if (edge_desc.type == EdgeType::Directed) {
                        has_directed = true;
                        break;
                    }
                }

                // Normalization factor
                // For undirected: divide by 2 (each path counted twice)
                // For directed: no division by 2
                // Both: divide by (n-1)*(n-2) for full normalization
                double normalization = has_directed ? 1.0 : 0.5;

                for (auto &[v, score] : betweenness) {
                    betweenness[v] = score * normalization;
                }

                return betweenness;
            }

            // Compute normalized betweenness centrality (scaled to [0, 1])
            template <typename VertexProperty>
            inline std::unordered_map<typename graphix::vertex::Graph<VertexProperty>::VertexId, double>
            betweenness_centrality_normalized(const graphix::vertex::Graph<VertexProperty> &graph) {

                auto betweenness = betweenness_centrality(graph);

                size_t n = graph.vertex_count();
                if (n <= 2) {
                    return betweenness;
                }

                double norm_factor = static_cast<double>((n - 1) * (n - 2));

                for (auto &[v, score] : betweenness) {
                    betweenness[v] = score / norm_factor;
                }

                return betweenness;
            }

            // ========================================================================
            // Utility Functions
            // ========================================================================

            // Find vertices with highest centrality score
            template <typename VertexId>
            inline std::vector<VertexId> top_k_central_vertices(const std::unordered_map<VertexId, double> &centrality,
                                                                size_t k) {

                // Create vector of (vertex, score) pairs
                std::vector<std::pair<VertexId, double>> pairs;
                pairs.reserve(centrality.size());
                for (const auto &[v, score] : centrality) {
                    pairs.push_back({v, score});
                }

                // Sort by score descending
                std::sort(pairs.begin(), pairs.end(), [](const auto &a, const auto &b) { return a.second > b.second; });

                // Take top k
                std::vector<VertexId> result;
                size_t count = std::min(k, pairs.size());
                result.reserve(count);
                for (size_t i = 0; i < count; ++i) {
                    result.push_back(pairs[i].first);
                }

                return result;
            }

            // Get vertex with highest centrality score
            template <typename VertexId>
            inline VertexId most_central_vertex(const std::unordered_map<VertexId, double> &centrality) {
                if (centrality.empty()) {
                    throw std::invalid_argument("Cannot find most central vertex in empty centrality map");
                }

                auto max_it = std::max_element(centrality.begin(), centrality.end(),
                                               [](const auto &a, const auto &b) { return a.second < b.second; });

                return max_it->first;
            }

        } // namespace algorithms
    } // namespace vertex
} // namespace graphix
