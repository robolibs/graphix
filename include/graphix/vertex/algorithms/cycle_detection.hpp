#pragma once

#include "graphix/vertex/graph.hpp"
#include <algorithm>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graphix {
    namespace vertex {

        // Cycle detection result
        struct CycleResult {
            bool has_cycle;
            std::vector<size_t> cycle; // Vertices forming the cycle (empty if no cycle)

            CycleResult() : has_cycle(false) {}
            CycleResult(bool cycle_found) : has_cycle(cycle_found) {}
            CycleResult(bool cycle_found, std::vector<size_t> cycle_verts)
                : has_cycle(cycle_found), cycle(std::move(cycle_verts)) {}
        };

        // ============================================================================
        // Directed Graph Cycle Detection
        // ============================================================================

        // Detect cycle in directed graph using DFS with three colors
        // White (unvisited), Gray (visiting), Black (visited)
        template <typename VertexProperty> CycleResult find_cycle_directed(const Graph<VertexProperty> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return CycleResult(false);
            }

            enum class State { Unvisited, Visiting, Visited };
            std::unordered_map<size_t, State> state;
            std::unordered_map<size_t, size_t> parent;
            std::vector<size_t> cycle;

            for (auto v : verts) {
                state[v] = State::Unvisited;
            }

            // DFS to find back edge (cycle)
            std::function<bool(size_t)> dfs = [&](size_t v) -> bool {
                state[v] = State::Visiting;

                auto neighbors = g.neighbors(v);
                for (auto neighbor : neighbors) {
                    // Only follow directed edges
                    auto edge_opt = g.get_edge(v, neighbor);
                    if (!edge_opt.has_value())
                        continue;

                    // Check if this edge is directed
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
                        // Found back edge - reconstruct cycle
                        cycle.push_back(neighbor);
                        size_t current = v;
                        while (current != neighbor && parent.find(current) != parent.end()) {
                            cycle.push_back(current);
                            current = parent[current];
                        }
                        if (current == neighbor) {
                            cycle.push_back(neighbor);
                        }
                        std::reverse(cycle.begin(), cycle.end());
                        return true;
                    } else if (state[neighbor] == State::Unvisited) {
                        parent[neighbor] = v;
                        if (dfs(neighbor)) {
                            return true;
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
                        return CycleResult(true, std::move(cycle));
                    }
                }
            }

            return CycleResult(false);
        }

        // Specialization for Graph<void>
        inline CycleResult find_cycle_directed(const Graph<void> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return CycleResult(false);
            }

            enum class State { Unvisited, Visiting, Visited };
            std::unordered_map<size_t, State> state;
            std::unordered_map<size_t, size_t> parent;
            std::vector<size_t> cycle;

            for (auto v : verts) {
                state[v] = State::Unvisited;
            }

            // DFS to find back edge (cycle)
            std::function<bool(size_t)> dfs = [&](size_t v) -> bool {
                state[v] = State::Visiting;

                auto neighbors = g.neighbors(v);
                for (auto neighbor : neighbors) {
                    // Only follow directed edges
                    auto edge_opt = g.get_edge(v, neighbor);
                    if (!edge_opt.has_value())
                        continue;

                    // Check if this edge is directed
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
                        // Found back edge - reconstruct cycle
                        cycle.push_back(neighbor);
                        size_t current = v;
                        while (current != neighbor && parent.find(current) != parent.end()) {
                            cycle.push_back(current);
                            current = parent[current];
                        }
                        if (current == neighbor) {
                            cycle.push_back(neighbor);
                        }
                        std::reverse(cycle.begin(), cycle.end());
                        return true;
                    } else if (state[neighbor] == State::Unvisited) {
                        parent[neighbor] = v;
                        if (dfs(neighbor)) {
                            return true;
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
                        return CycleResult(true, std::move(cycle));
                    }
                }
            }

            return CycleResult(false);
        }

        // ============================================================================
        // Undirected Graph Cycle Detection
        // ============================================================================

        // Detect cycle in undirected graph using DFS
        // A cycle exists if we find an edge to a visited vertex that's not the parent
        template <typename VertexProperty> CycleResult find_cycle_undirected(const Graph<VertexProperty> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return CycleResult(false);
            }

            std::unordered_set<size_t> visited;
            std::unordered_map<size_t, size_t> parent;
            std::vector<size_t> cycle;

            // DFS to find cycle
            std::function<bool(size_t, size_t)> dfs = [&](size_t v, size_t par) -> bool {
                visited.insert(v);
                parent[v] = par;

                auto neighbors = g.neighbors(v);
                for (auto neighbor : neighbors) {
                    // Only follow undirected edges
                    auto edge_opt = g.get_edge(v, neighbor);
                    if (!edge_opt.has_value())
                        continue;

                    // Check if this edge is undirected
                    auto edges = g.edges();
                    bool is_undirected = false;
                    for (const auto &e : edges) {
                        if (e.id == edge_opt.value() && e.type == EdgeType::Undirected) {
                            is_undirected = true;
                            break;
                        }
                    }

                    if (!is_undirected)
                        continue;

                    if (visited.find(neighbor) != visited.end()) {
                        // Found visited vertex
                        if (neighbor != par) {
                            // Cycle found - reconstruct it
                            cycle.push_back(neighbor);
                            size_t current = v;
                            while (current != neighbor) {
                                cycle.push_back(current);
                                current = parent[current];
                            }
                            cycle.push_back(neighbor);
                            std::reverse(cycle.begin(), cycle.end());
                            return true;
                        }
                    } else {
                        if (dfs(neighbor, v)) {
                            return true;
                        }
                    }
                }

                return false;
            };

            // Try DFS from all unvisited vertices
            for (auto v : verts) {
                if (visited.find(v) == visited.end()) {
                    if (dfs(v, v)) { // Use v as its own parent for root
                        return CycleResult(true, std::move(cycle));
                    }
                }
            }

            return CycleResult(false);
        }

        // Specialization for Graph<void>
        inline CycleResult find_cycle_undirected(const Graph<void> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return CycleResult(false);
            }

            std::unordered_set<size_t> visited;
            std::unordered_map<size_t, size_t> parent;
            std::vector<size_t> cycle;

            // DFS to find cycle
            std::function<bool(size_t, size_t)> dfs = [&](size_t v, size_t par) -> bool {
                visited.insert(v);
                parent[v] = par;

                auto neighbors = g.neighbors(v);
                for (auto neighbor : neighbors) {
                    // Only follow undirected edges
                    auto edge_opt = g.get_edge(v, neighbor);
                    if (!edge_opt.has_value())
                        continue;

                    // Check if this edge is undirected
                    auto edges = g.edges();
                    bool is_undirected = false;
                    for (const auto &e : edges) {
                        if (e.id == edge_opt.value() && e.type == EdgeType::Undirected) {
                            is_undirected = true;
                            break;
                        }
                    }

                    if (!is_undirected)
                        continue;

                    if (visited.find(neighbor) != visited.end()) {
                        // Found visited vertex
                        if (neighbor != par) {
                            // Cycle found - reconstruct it
                            cycle.push_back(neighbor);
                            size_t current = v;
                            while (current != neighbor) {
                                cycle.push_back(current);
                                current = parent[current];
                            }
                            cycle.push_back(neighbor);
                            std::reverse(cycle.begin(), cycle.end());
                            return true;
                        }
                    } else {
                        if (dfs(neighbor, v)) {
                            return true;
                        }
                    }
                }

                return false;
            };

            // Try DFS from all unvisited vertices
            for (auto v : verts) {
                if (visited.find(v) == visited.end()) {
                    if (dfs(v, v)) { // Use v as its own parent for root
                        return CycleResult(true, std::move(cycle));
                    }
                }
            }

            return CycleResult(false);
        }

        // ============================================================================
        // General Cycle Detection (Auto-detect graph type)
        // ============================================================================

        // Automatically detect whether graph is directed or undirected and use appropriate algorithm
        template <typename VertexProperty> CycleResult find_cycle(const Graph<VertexProperty> &g) {
            auto edges = g.edges();
            if (edges.empty()) {
                return CycleResult(false);
            }

            // Check if graph has any directed edges
            bool has_directed = false;
            bool has_undirected = false;

            for (const auto &edge : edges) {
                if (edge.type == EdgeType::Directed) {
                    has_directed = true;
                } else {
                    has_undirected = true;
                }
            }

            // If mixed or purely directed, use directed algorithm
            if (has_directed) {
                return find_cycle_directed(g);
            }
            // Otherwise use undirected algorithm
            else {
                return find_cycle_undirected(g);
            }
        }

        // Specialization for Graph<void>
        inline CycleResult find_cycle(const Graph<void> &g) {
            auto edges = g.edges();
            if (edges.empty()) {
                return CycleResult(false);
            }

            // Check if graph has any directed edges
            bool has_directed = false;
            bool has_undirected = false;

            for (const auto &edge : edges) {
                if (edge.type == EdgeType::Directed) {
                    has_directed = true;
                } else {
                    has_undirected = true;
                }
            }

            // If mixed or purely directed, use directed algorithm
            if (has_directed) {
                return find_cycle_directed(g);
            }
            // Otherwise use undirected algorithm
            else {
                return find_cycle_undirected(g);
            }
        }

        // ============================================================================
        // Simple Boolean Check
        // ============================================================================

        // Simple boolean check - does graph have a cycle?
        template <typename VertexProperty> inline bool has_cycle(const Graph<VertexProperty> &g) {
            return find_cycle(g).has_cycle;
        }

        // Specialization for Graph<void>
        inline bool has_cycle(const Graph<void> &g) { return find_cycle(g).has_cycle; }

        // Directed-specific check
        template <typename VertexProperty> inline bool has_cycle_directed(const Graph<VertexProperty> &g) {
            return find_cycle_directed(g).has_cycle;
        }

        inline bool has_cycle_directed(const Graph<void> &g) { return find_cycle_directed(g).has_cycle; }

        // Undirected-specific check
        template <typename VertexProperty> inline bool has_cycle_undirected(const Graph<VertexProperty> &g) {
            return find_cycle_undirected(g).has_cycle;
        }

        inline bool has_cycle_undirected(const Graph<void> &g) { return find_cycle_undirected(g).has_cycle; }

    } // namespace vertex
} // namespace graphix
