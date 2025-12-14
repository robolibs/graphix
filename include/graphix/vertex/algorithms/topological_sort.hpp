#pragma once

#include "graphix/vertex/graph.hpp"
#include <algorithm>
#include <functional>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graphix {
    namespace vertex {

        // Topological sort result
        struct TopologicalSortResult {
            std::vector<size_t> order; // Topologically sorted vertices
            bool is_dag;               // True if graph is a DAG (no cycles)
            std::vector<size_t> cycle; // If not DAG, contains a cycle (may be empty)

            TopologicalSortResult() : is_dag(true) {}
            TopologicalSortResult(std::vector<size_t> ord) : order(std::move(ord)), is_dag(true) {}
            TopologicalSortResult(bool dag, std::vector<size_t> cyc = {}) : is_dag(dag), cycle(std::move(cyc)) {}
        };

        // DFS-based topological sort using Kahn's algorithm variant
        // Returns vertices in topological order if graph is a DAG
        // Returns empty order with is_dag=false if graph has cycles
        template <typename VertexProperty> TopologicalSortResult topological_sort(const Graph<VertexProperty> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return TopologicalSortResult();
            }

            // Build in-degree map for all vertices
            std::unordered_map<size_t, size_t> in_degree;
            for (auto v : verts) {
                in_degree[v] = 0;
            }

            // Count in-degrees by examining all edges
            auto all_edges = g.edges();
            for (const auto &edge : all_edges) {
                // Only count directed edges
                if (edge.type == EdgeType::Directed) {
                    in_degree[edge.target]++;
                }
            }

            // Find all vertices with in-degree 0 (starting points)
            std::vector<size_t> queue;
            for (auto v : verts) {
                if (in_degree[v] == 0) {
                    queue.push_back(v);
                }
            }

            std::vector<size_t> result;
            result.reserve(verts.size());

            // Process vertices with in-degree 0
            while (!queue.empty()) {
                // Pop from queue
                size_t current = queue.back();
                queue.pop_back();
                result.push_back(current);

                // Reduce in-degree of neighbors
                for (const auto &edge : all_edges) {
                    if (edge.source == current && edge.type == EdgeType::Directed) {
                        in_degree[edge.target]--;
                        if (in_degree[edge.target] == 0) {
                            queue.push_back(edge.target);
                        }
                    }
                }
            }

            // If we processed all vertices, it's a DAG
            if (result.size() == verts.size()) {
                return TopologicalSortResult(std::move(result));
            }

            // Otherwise, there's a cycle - find it using DFS
            std::vector<size_t> cycle = find_cycle_dfs(g, in_degree);
            return TopologicalSortResult(false, std::move(cycle));
        }

        // Specialization for Graph<void>
        inline TopologicalSortResult topological_sort(const Graph<void> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return TopologicalSortResult();
            }

            // Build in-degree map for all vertices
            std::unordered_map<size_t, size_t> in_degree;
            for (auto v : verts) {
                in_degree[v] = 0;
            }

            // Count in-degrees by examining all edges
            auto all_edges = g.edges();
            for (const auto &edge : all_edges) {
                // Only count directed edges
                if (edge.type == EdgeType::Directed) {
                    in_degree[edge.target]++;
                }
            }

            // Find all vertices with in-degree 0 (starting points)
            std::vector<size_t> queue;
            for (auto v : verts) {
                if (in_degree[v] == 0) {
                    queue.push_back(v);
                }
            }

            std::vector<size_t> result;
            result.reserve(verts.size());

            // Process vertices with in-degree 0
            while (!queue.empty()) {
                // Pop from queue
                size_t current = queue.back();
                queue.pop_back();
                result.push_back(current);

                // Reduce in-degree of neighbors
                for (const auto &edge : all_edges) {
                    if (edge.source == current && edge.type == EdgeType::Directed) {
                        in_degree[edge.target]--;
                        if (in_degree[edge.target] == 0) {
                            queue.push_back(edge.target);
                        }
                    }
                }
            }

            // If we processed all vertices, it's a DAG
            if (result.size() == verts.size()) {
                return TopologicalSortResult(std::move(result));
            }

            // Otherwise, there's a cycle
            return TopologicalSortResult(false);
        }

        // Helper function to find a cycle using DFS (for detailed cycle reporting)
        template <typename VertexProperty>
        std::vector<size_t> find_cycle_dfs(const Graph<VertexProperty> &g,
                                           const std::unordered_map<size_t, size_t> &remaining_in_degree) {
            enum class State { Unvisited, Visiting, Visited };
            std::unordered_map<size_t, State> state;
            std::unordered_map<size_t, size_t> parent;
            std::vector<size_t> cycle;

            auto verts = g.vertices();
            for (auto v : verts) {
                state[v] = State::Unvisited;
            }

            // DFS function to find cycle
            std::function<bool(size_t)> dfs = [&](size_t v) -> bool {
                state[v] = State::Visiting;

                auto neighbors = g.neighbors(v);
                for (auto neighbor : neighbors) {
                    // Only follow directed edges
                    auto edge_opt = g.get_edge(v, neighbor);
                    if (edge_opt.has_value()) {
                        auto edges = g.edges();
                        bool is_directed = false;
                        for (const auto &e : edges) {
                            if (e.id == edge_opt.value() && e.type == EdgeType::Directed) {
                                is_directed = true;
                                break;
                            }
                        }

                        if (!is_directed)
                            continue;

                        if (state[neighbor] == State::Visiting) {
                            // Found a cycle - reconstruct it
                            cycle.push_back(neighbor);
                            size_t current = v;
                            while (current != neighbor) {
                                cycle.push_back(current);
                                current = parent[current];
                            }
                            cycle.push_back(neighbor);
                            std::reverse(cycle.begin(), cycle.end());
                            return true;
                        } else if (state[neighbor] == State::Unvisited) {
                            parent[neighbor] = v;
                            if (dfs(neighbor)) {
                                return true;
                            }
                        }
                    }
                }

                state[v] = State::Visited;
                return false;
            };

            // Try DFS from all unvisited vertices
            for (auto v : verts) {
                if (state[v] == State::Unvisited) {
                    if (dfs(v)) {
                        return cycle;
                    }
                }
            }

            return cycle;
        }

        // Alternative: DFS-based topological sort (produces different but valid ordering)
        // This uses the classic DFS postorder approach
        template <typename VertexProperty> TopologicalSortResult topological_sort_dfs(const Graph<VertexProperty> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return TopologicalSortResult();
            }

            enum class State { Unvisited, Visiting, Visited };
            std::unordered_map<size_t, State> state;
            std::stack<size_t> postorder;
            bool has_cycle = false;

            for (auto v : verts) {
                state[v] = State::Unvisited;
            }

            // DFS function
            std::function<void(size_t)> dfs = [&](size_t v) {
                if (has_cycle)
                    return;

                state[v] = State::Visiting;

                auto neighbors = g.neighbors(v);
                for (auto neighbor : neighbors) {
                    // Check if this is a directed edge
                    auto edge_opt = g.get_edge(v, neighbor);
                    if (edge_opt.has_value()) {
                        auto edges = g.edges();
                        bool is_directed = false;
                        for (const auto &e : edges) {
                            if (e.id == edge_opt.value() && e.type == EdgeType::Directed) {
                                is_directed = true;
                                break;
                            }
                        }

                        if (!is_directed)
                            continue;

                        if (state[neighbor] == State::Visiting) {
                            // Back edge = cycle
                            has_cycle = true;
                            return;
                        } else if (state[neighbor] == State::Unvisited) {
                            dfs(neighbor);
                        }
                    }
                }

                state[v] = State::Visited;
                postorder.push(v);
            };

            // Visit all vertices
            for (auto v : verts) {
                if (state[v] == State::Unvisited) {
                    dfs(v);
                    if (has_cycle) {
                        return TopologicalSortResult(false);
                    }
                }
            }

            // Reverse postorder gives topological order
            std::vector<size_t> result;
            result.reserve(verts.size());
            while (!postorder.empty()) {
                result.push_back(postorder.top());
                postorder.pop();
            }

            return TopologicalSortResult(std::move(result));
        }

        // Specialization for Graph<void>
        inline TopologicalSortResult topological_sort_dfs(const Graph<void> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return TopologicalSortResult();
            }

            enum class State { Unvisited, Visiting, Visited };
            std::unordered_map<size_t, State> state;
            std::stack<size_t> postorder;
            bool has_cycle = false;

            for (auto v : verts) {
                state[v] = State::Unvisited;
            }

            // DFS function
            std::function<void(size_t)> dfs = [&](size_t v) {
                if (has_cycle)
                    return;

                state[v] = State::Visiting;

                auto neighbors = g.neighbors(v);
                for (auto neighbor : neighbors) {
                    // Check if this is a directed edge
                    auto edge_opt = g.get_edge(v, neighbor);
                    if (edge_opt.has_value()) {
                        auto edges = g.edges();
                        bool is_directed = false;
                        for (const auto &e : edges) {
                            if (e.id == edge_opt.value() && e.type == EdgeType::Directed) {
                                is_directed = true;
                                break;
                            }
                        }

                        if (!is_directed)
                            continue;

                        if (state[neighbor] == State::Visiting) {
                            // Back edge = cycle
                            has_cycle = true;
                            return;
                        } else if (state[neighbor] == State::Unvisited) {
                            dfs(neighbor);
                        }
                    }
                }

                state[v] = State::Visited;
                postorder.push(v);
            };

            // Visit all vertices
            for (auto v : verts) {
                if (state[v] == State::Unvisited) {
                    dfs(v);
                    if (has_cycle) {
                        return TopologicalSortResult(false);
                    }
                }
            }

            // Reverse postorder gives topological order
            std::vector<size_t> result;
            result.reserve(verts.size());
            while (!postorder.empty()) {
                result.push_back(postorder.top());
                postorder.pop();
            }

            return TopologicalSortResult(std::move(result));
        }

    } // namespace vertex
} // namespace graphix
