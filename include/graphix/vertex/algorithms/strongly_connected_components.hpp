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

        // Strongly Connected Components result
        struct SCCResult {
            std::vector<std::vector<size_t>> components; // Each component is a list of vertices
            size_t num_components;                       // Number of SCCs found

            SCCResult() : num_components(0) {}
            SCCResult(std::vector<std::vector<size_t>> comps)
                : components(std::move(comps)), num_components(components.size()) {}
        };

        // ============================================================================
        // Tarjan's Algorithm for Strongly Connected Components
        // ============================================================================
        // Uses a single DFS pass with low-link values
        // Time complexity: O(V + E)
        // Space complexity: O(V)

        template <typename VertexProperty> SCCResult strongly_connected_components(const Graph<VertexProperty> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return SCCResult();
            }

            // Tarjan's algorithm state
            std::unordered_map<size_t, size_t> index;    // Discovery time of vertex
            std::unordered_map<size_t, size_t> lowlink;  // Lowest index reachable from vertex
            std::unordered_set<size_t> on_stack;         // Track vertices on stack
            std::stack<size_t> stack;                    // DFS stack
            size_t current_index = 0;                    // Current discovery index
            std::vector<std::vector<size_t>> components; // Result SCCs

            // Tarjan's DFS
            std::function<void(size_t)> strongconnect = [&](size_t v) {
                // Set the depth index for v
                index[v] = current_index;
                lowlink[v] = current_index;
                current_index++;
                stack.push(v);
                on_stack.insert(v);

                // Consider successors of v (only via directed edges)
                auto neighbors = g.neighbors(v);
                for (auto w : neighbors) {
                    // Only follow directed edges
                    auto edge_opt = g.get_edge(v, w);
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

                    if (index.find(w) == index.end()) {
                        // Successor w has not yet been visited; recurse on it
                        strongconnect(w);
                        lowlink[v] = std::min(lowlink[v], lowlink[w]);
                    } else if (on_stack.find(w) != on_stack.end()) {
                        // Successor w is on stack and hence in the current SCC
                        lowlink[v] = std::min(lowlink[v], index[w]);
                    }
                }

                // If v is a root node, pop the stack and create an SCC
                if (lowlink[v] == index[v]) {
                    std::vector<size_t> component;
                    size_t w;
                    do {
                        w = stack.top();
                        stack.pop();
                        on_stack.erase(w);
                        component.push_back(w);
                    } while (w != v);

                    // Sort component for deterministic output
                    std::sort(component.begin(), component.end());
                    components.push_back(std::move(component));
                }
            };

            // Run Tarjan's algorithm from all unvisited vertices
            for (auto v : verts) {
                if (index.find(v) == index.end()) {
                    strongconnect(v);
                }
            }

            // Sort components by their first (smallest) vertex for deterministic output
            std::sort(components.begin(), components.end(),
                      [](const std::vector<size_t> &a, const std::vector<size_t> &b) {
                          return !a.empty() && !b.empty() && a[0] < b[0];
                      });

            return SCCResult(std::move(components));
        }

        // Specialization for Graph<void>
        inline SCCResult strongly_connected_components(const Graph<void> &g) {
            auto verts = g.vertices();
            if (verts.empty()) {
                return SCCResult();
            }

            // Tarjan's algorithm state
            std::unordered_map<size_t, size_t> index;    // Discovery time of vertex
            std::unordered_map<size_t, size_t> lowlink;  // Lowest index reachable from vertex
            std::unordered_set<size_t> on_stack;         // Track vertices on stack
            std::stack<size_t> stack;                    // DFS stack
            size_t current_index = 0;                    // Current discovery index
            std::vector<std::vector<size_t>> components; // Result SCCs

            // Tarjan's DFS
            std::function<void(size_t)> strongconnect = [&](size_t v) {
                // Set the depth index for v
                index[v] = current_index;
                lowlink[v] = current_index;
                current_index++;
                stack.push(v);
                on_stack.insert(v);

                // Consider successors of v (only via directed edges)
                auto neighbors = g.neighbors(v);
                for (auto w : neighbors) {
                    // Only follow directed edges
                    auto edge_opt = g.get_edge(v, w);
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

                    if (index.find(w) == index.end()) {
                        // Successor w has not yet been visited; recurse on it
                        strongconnect(w);
                        lowlink[v] = std::min(lowlink[v], lowlink[w]);
                    } else if (on_stack.find(w) != on_stack.end()) {
                        // Successor w is on stack and hence in the current SCC
                        lowlink[v] = std::min(lowlink[v], index[w]);
                    }
                }

                // If v is a root node, pop the stack and create an SCC
                if (lowlink[v] == index[v]) {
                    std::vector<size_t> component;
                    size_t w;
                    do {
                        w = stack.top();
                        stack.pop();
                        on_stack.erase(w);
                        component.push_back(w);
                    } while (w != v);

                    // Sort component for deterministic output
                    std::sort(component.begin(), component.end());
                    components.push_back(std::move(component));
                }
            };

            // Run Tarjan's algorithm from all unvisited vertices
            for (auto v : verts) {
                if (index.find(v) == index.end()) {
                    strongconnect(v);
                }
            }

            // Sort components by their first (smallest) vertex for deterministic output
            std::sort(components.begin(), components.end(),
                      [](const std::vector<size_t> &a, const std::vector<size_t> &b) {
                          return !a.empty() && !b.empty() && a[0] < b[0];
                      });

            return SCCResult(std::move(components));
        }

        // ============================================================================
        // Helper Functions
        // ============================================================================

        // Check if graph is strongly connected (has exactly one SCC)
        template <typename VertexProperty> inline bool is_strongly_connected(const Graph<VertexProperty> &g) {
            auto result = strongly_connected_components(g);
            return result.num_components == 1 && !result.components.empty();
        }

        inline bool is_strongly_connected(const Graph<void> &g) {
            auto result = strongly_connected_components(g);
            return result.num_components == 1 && !result.components.empty();
        }

        // Get the component ID for each vertex
        // Returns a map: vertex -> component_id
        template <typename VertexProperty>
        std::unordered_map<size_t, size_t> get_component_map(const Graph<VertexProperty> &g) {
            auto result = strongly_connected_components(g);
            std::unordered_map<size_t, size_t> component_map;

            for (size_t comp_id = 0; comp_id < result.components.size(); comp_id++) {
                for (auto v : result.components[comp_id]) {
                    component_map[v] = comp_id;
                }
            }

            return component_map;
        }

        inline std::unordered_map<size_t, size_t> get_component_map(const Graph<void> &g) {
            auto result = strongly_connected_components(g);
            std::unordered_map<size_t, size_t> component_map;

            for (size_t comp_id = 0; comp_id < result.components.size(); comp_id++) {
                for (auto v : result.components[comp_id]) {
                    component_map[v] = comp_id;
                }
            }

            return component_map;
        }

        // Get size of largest SCC
        template <typename VertexProperty> inline size_t largest_scc_size(const Graph<VertexProperty> &g) {
            auto result = strongly_connected_components(g);
            size_t max_size = 0;
            for (const auto &comp : result.components) {
                max_size = std::max(max_size, comp.size());
            }
            return max_size;
        }

        inline size_t largest_scc_size(const Graph<void> &g) {
            auto result = strongly_connected_components(g);
            size_t max_size = 0;
            for (const auto &comp : result.components) {
                max_size = std::max(max_size, comp.size());
            }
            return max_size;
        }

        // Check if two vertices are in the same SCC
        template <typename VertexProperty> inline bool in_same_scc(const Graph<VertexProperty> &g, size_t u, size_t v) {
            auto comp_map = get_component_map(g);
            return comp_map.find(u) != comp_map.end() && comp_map.find(v) != comp_map.end() &&
                   comp_map[u] == comp_map[v];
        }

        inline bool in_same_scc(const Graph<void> &g, size_t u, size_t v) {
            auto comp_map = get_component_map(g);
            return comp_map.find(u) != comp_map.end() && comp_map.find(v) != comp_map.end() &&
                   comp_map[u] == comp_map[v];
        }

    } // namespace vertex
} // namespace graphix
