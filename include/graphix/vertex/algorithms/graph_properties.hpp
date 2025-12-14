#pragma once

#include "graphix/vertex/graph.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector> // namespace graphix::vertex

namespace graphix::vertex::algorithms {

    // ========================================================================
    // is_bipartite - Check if graph can be 2-colored (no odd cycles)
    // ========================================================================

    // Result of bipartite check
    template <typename VertexId> struct BipartiteResult {
        bool is_bipartite = false;
        std::unordered_map<VertexId, int> coloring; // 0 or 1 for each vertex (if bipartite)
        std::vector<VertexId> odd_cycle;            // cycle that proves non-bipartiteness (if any)
    };

    // Check if graph is bipartite using BFS-based 2-coloring
    // A graph is bipartite if and only if it contains no odd-length cycles
    template <typename VertexProperty>
    inline BipartiteResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    is_bipartite(const graphix::vertex::Graph<VertexProperty> &graph) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        BipartiteResult<VertexId> result;
        result.is_bipartite = true;

        // Try to 2-color each connected component
        for (auto start : graph.vertices()) {
            // Skip if already colored
            if (result.coloring.count(start)) {
                continue;
            }

            // BFS with coloring
            std::queue<VertexId> queue;
            std::unordered_map<VertexId, VertexId> parent;

            queue.push(start);
            result.coloring[start] = 0;
            parent[start] = start;

            while (!queue.empty()) {
                auto current = queue.front();
                queue.pop();

                int current_color = result.coloring[current];
                int next_color = 1 - current_color;

                for (auto neighbor : graph.neighbors(current)) {
                    if (!result.coloring.count(neighbor)) {
                        // Not colored yet - assign opposite color
                        result.coloring[neighbor] = next_color;
                        parent[neighbor] = current;
                        queue.push(neighbor);
                    } else if (result.coloring[neighbor] == current_color) {
                        // Same color as current - found odd cycle!
                        result.is_bipartite = false;

                        // Reconstruct the odd cycle
                        std::unordered_set<VertexId> path_from_current;
                        std::vector<VertexId> path_from_neighbor;

                        // Trace back from current
                        VertexId v = current;
                        while (v != start) {
                            path_from_current.insert(v);
                            v = parent[v];
                        }
                        path_from_current.insert(start);

                        // Trace back from neighbor until we hit path_from_current
                        v = neighbor;
                        while (!path_from_current.count(v)) {
                            path_from_neighbor.push_back(v);
                            v = parent[v];
                        }
                        path_from_neighbor.push_back(v); // common ancestor

                        // Build cycle: common_ancestor -> ... -> current -> neighbor -> ... ->
                        // common_ancestor
                        VertexId common = v;
                        result.odd_cycle.clear();

                        // From common to current
                        std::vector<VertexId> path1;
                        v = current;
                        while (v != common) {
                            path1.push_back(v);
                            v = parent[v];
                        }
                        path1.push_back(common);
                        std::reverse(path1.begin(), path1.end());

                        // Combine: common -> current -> neighbor -> common
                        result.odd_cycle = path1;
                        result.odd_cycle.push_back(neighbor);
                        for (int i = static_cast<int>(path_from_neighbor.size()) - 2; i >= 0; --i) {
                            result.odd_cycle.push_back(path_from_neighbor[i]);
                        }

                        return result;
                    }
                }
            }
        }

        return result;
    }

    // ========================================================================
    // is_acyclic - Check if graph has no cycles
    // ========================================================================

    // Check if directed graph is acyclic (is a DAG)
    // Uses DFS with three colors: white (unvisited), gray (in progress), black (finished)
    template <typename VertexProperty>
    inline bool is_acyclic_directed(const graphix::vertex::Graph<VertexProperty> &graph) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        enum class Color { WHITE, GRAY, BLACK };

        std::unordered_map<VertexId, Color> color;

        // Initialize all vertices as unvisited
        for (auto v : graph.vertices()) {
            color[v] = Color::WHITE;
        }

        // DFS helper with cycle detection
        auto has_cycle_dfs = [&](auto &self, VertexId v) -> bool {
            color[v] = Color::GRAY; // Mark as in-progress

            for (auto edge_id : graph.out_edges(v)) {
                // Only consider directed edges
                if (graph.get_edge_type(edge_id) != graphix::vertex::EdgeType::Directed) {
                    continue;
                }

                VertexId neighbor = graph.target(edge_id);
                if (neighbor == v) {
                    neighbor = graph.source(edge_id);
                }

                if (color[neighbor] == Color::GRAY) {
                    // Back edge found - cycle detected
                    return true;
                } else if (color[neighbor] == Color::WHITE) {
                    if (self(self, neighbor)) {
                        return true;
                    }
                }
            }

            color[v] = Color::BLACK; // Mark as finished
            return false;
        };

        // Check each component
        for (auto v : graph.vertices()) {
            if (color[v] == Color::WHITE) {
                if (has_cycle_dfs(has_cycle_dfs, v)) {
                    return false;
                }
            }
        }

        return true;
    }

    // Check if undirected graph is acyclic (is a forest/tree)
    // An undirected graph is acyclic if it has no cycles
    template <typename VertexProperty>
    inline bool is_acyclic_undirected(const graphix::vertex::Graph<VertexProperty> &graph) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        std::unordered_set<VertexId> visited;

        // DFS with parent tracking
        auto has_cycle_dfs = [&](auto &self, VertexId v, VertexId parent) -> bool {
            visited.insert(v);

            for (auto neighbor : graph.neighbors(v)) {
                // Self-loop check (v -> v)
                if (neighbor == v) {
                    return true;
                }

                // Skip parent edge
                if (neighbor == parent) {
                    continue;
                }

                if (visited.count(neighbor)) {
                    // Already visited and not parent - cycle found
                    return true;
                }

                if (self(self, neighbor, v)) {
                    return true;
                }
            }

            return false;
        };

        // Check each component
        for (auto v : graph.vertices()) {
            if (!visited.count(v)) {
                VertexId no_parent = v; // Use vertex itself as sentinel for "no parent"
                if (has_cycle_dfs(has_cycle_dfs, v, no_parent)) {
                    return false;
                }
            }
        }

        return true;
    }

    // Unified is_acyclic that handles both directed and undirected graphs
    template <typename VertexProperty> inline bool is_acyclic(const graphix::vertex::Graph<VertexProperty> &graph) {
        // Check if graph has any directed edges
        bool has_directed = false;
        bool has_undirected = false;

        for (auto v : graph.vertices()) {
            for (auto edge_id : graph.out_edges(v)) {
                if (graph.get_edge_type(edge_id) == EdgeType::Directed) {
                    has_directed = true;
                } else {
                    has_undirected = true;
                }
                if (has_directed && has_undirected)
                    break;
            }
            if (has_directed && has_undirected)
                break;
        }

        // If pure directed, use directed algorithm
        if (has_directed && !has_undirected) {
            return is_acyclic_directed(graph);
        }
        // If pure undirected or mixed, use undirected algorithm
        else {
            return is_acyclic_undirected(graph);
        }
    }

    // ========================================================================
    // graph_diameter - Longest shortest path in graph
    // ========================================================================

    // Compute the diameter of the graph (longest shortest path)
    // Returns infinity if graph is disconnected
    template <typename VertexProperty>
    inline double graph_diameter(const graphix::vertex::Graph<VertexProperty> &graph) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        if (graph.vertex_count() == 0) {
            return 0.0;
        }

        double max_distance = 0.0;
        bool is_connected = true;

        // Run BFS from each vertex to find eccentricity
        for (auto source : graph.vertices()) {
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
                        distance[neighbor] = distance[current] + graph.get_weight(edge_id);
                        queue.push(neighbor);
                    }
                }
            }

            // Check if all vertices were reached
            if (visited.size() != graph.vertex_count()) {
                is_connected = false;
                break;
            }

            // Find max distance from this source (eccentricity)
            for (auto [v, dist] : distance) {
                max_distance = std::max(max_distance, dist);
            }
        }

        if (!is_connected) {
            return std::numeric_limits<double>::infinity();
        }

        return max_distance;
    }

    // Compute radius of graph (minimum eccentricity)
    template <typename VertexProperty> inline double graph_radius(const graphix::vertex::Graph<VertexProperty> &graph) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        if (graph.vertex_count() == 0) {
            return 0.0;
        }

        double min_eccentricity = std::numeric_limits<double>::infinity();

        // Run BFS from each vertex to find eccentricity
        for (auto source : graph.vertices()) {
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
                        distance[neighbor] = distance[current] + graph.get_weight(edge_id);
                        queue.push(neighbor);
                    }
                }
            }

            // Check if all vertices were reached
            if (visited.size() != graph.vertex_count()) {
                return std::numeric_limits<double>::infinity();
            }

            // Find max distance from this source (eccentricity)
            double eccentricity = 0.0;
            for (auto [v, dist] : distance) {
                eccentricity = std::max(eccentricity, dist);
            }

            min_eccentricity = std::min(min_eccentricity, eccentricity);
        }

        return min_eccentricity;
    }

    // Find center vertices (vertices with minimum eccentricity)
    template <typename VertexProperty>
    inline std::vector<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    graph_center(const graphix::vertex::Graph<VertexProperty> &graph) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        std::vector<VertexId> center;
        double radius = graph_radius(graph);

        if (std::isinf(radius)) {
            return center; // Empty for disconnected graphs
        }

        // Find all vertices with eccentricity == radius
        for (auto source : graph.vertices()) {
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
                        distance[neighbor] = distance[current] + graph.get_weight(edge_id);
                        queue.push(neighbor);
                    }
                }
            }

            // Calculate eccentricity
            double eccentricity = 0.0;
            for (auto [v, dist] : distance) {
                eccentricity = std::max(eccentricity, dist);
            }

            // Check if this is a center vertex
            if (std::abs(eccentricity - radius) < 1e-9) {
                center.push_back(source);
            }
        }

        return center;
    }

} // namespace graphix::vertex::algorithms
