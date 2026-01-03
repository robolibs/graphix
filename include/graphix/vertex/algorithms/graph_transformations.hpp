#pragma once

#include "graphix/vertex/graph.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graphix {
    namespace vertex {
        namespace algorithms {

            // Simple hash function for pairs (for edge lookup)
            struct PairHash {
                template <typename T1, typename T2> std::size_t operator()(const std::pair<T1, T2> &p) const {
                    auto h1 = std::hash<T1>{}(p.first);
                    auto h2 = std::hash<T2>{}(p.second);
                    return h1 ^ (h2 << 1);
                }
            };

            // ========================================================================
            // Graph Transpose/Reverse - Reverse all edge directions
            // ========================================================================

            // Create a transposed graph (all edges reversed)
            // For directed edges: u -> v becomes v -> u
            // For undirected edges: edges remain unchanged
            // Vertex properties are preserved
            template <typename VertexProperty>
            inline Graph<VertexProperty> transpose(const Graph<VertexProperty> &graph) {
                Graph<VertexProperty> result;

                // Copy all vertices with their properties
                std::unordered_map<typename Graph<VertexProperty>::VertexId, typename Graph<VertexProperty>::VertexId>
                    old_to_new;

                for (auto v : graph.vertices()) {
                    typename Graph<VertexProperty>::VertexId new_v;
                    if constexpr (std::is_same_v<VertexProperty, void>) {
                        new_v = result.add_vertex();
                    } else {
                        new_v = result.add_vertex(graph[v]);
                    }
                    old_to_new[v] = new_v;
                }

                // Add edges with reversed direction (for directed edges)
                for (const auto &edge_desc : graph.edges()) {
                    auto src = old_to_new[edge_desc.source];
                    auto tgt = old_to_new[edge_desc.target];

                    if (edge_desc.type == EdgeType::Directed) {
                        // Reverse directed edges: u -> v becomes v -> u
                        result.add_edge(tgt, src, edge_desc.weight, EdgeType::Directed);
                    } else {
                        // Undirected edges remain undirected
                        result.add_edge(src, tgt, edge_desc.weight, EdgeType::Undirected);
                    }
                }

                return result;
            }

            // Specialization for void (no vertex properties)
            inline Graph<void> transpose(const Graph<void> &graph) {
                Graph<void> result;

                // Create mapping from old to new vertex IDs
                std::unordered_map<Graph<void>::VertexId, Graph<void>::VertexId> old_to_new;

                for (auto v : graph.vertices()) {
                    auto new_v = result.add_vertex();
                    old_to_new[v] = new_v;
                }

                // Add edges with reversed direction (for directed edges)
                for (const auto &edge_desc : graph.edges()) {
                    auto src = old_to_new[edge_desc.source];
                    auto tgt = old_to_new[edge_desc.target];

                    if (edge_desc.type == EdgeType::Directed) {
                        // Reverse directed edges: u -> v becomes v -> u
                        result.add_edge(tgt, src, edge_desc.weight, EdgeType::Directed);
                    } else {
                        // Undirected edges remain undirected
                        result.add_edge(src, tgt, edge_desc.weight, EdgeType::Undirected);
                    }
                }

                return result;
            }

            // ========================================================================
            // Subgraph Extraction - Extract subgraph from vertex set
            // ========================================================================

            // Extract induced subgraph from a set of vertices
            // An induced subgraph contains all edges between vertices in the set
            template <typename VertexProperty>
            inline Graph<VertexProperty>
            induced_subgraph(const Graph<VertexProperty> &graph,
                             const std::unordered_set<typename Graph<VertexProperty>::VertexId> &vertex_set) {

                using VertexId = typename Graph<VertexProperty>::VertexId;
                Graph<VertexProperty> result;

                // Map old vertex IDs to new vertex IDs
                std::unordered_map<VertexId, VertexId> old_to_new;

                // Add vertices that are in the set
                for (auto v : graph.vertices()) {
                    if (vertex_set.count(v)) {
                        VertexId new_v;
                        if constexpr (std::is_same_v<VertexProperty, void>) {
                            new_v = result.add_vertex();
                        } else {
                            new_v = result.add_vertex(graph[v]);
                        }
                        old_to_new[v] = new_v;
                    }
                }

                // Add edges where both endpoints are in the vertex set
                for (const auto &edge_desc : graph.edges()) {
                    if (vertex_set.count(edge_desc.source) && vertex_set.count(edge_desc.target)) {
                        auto src = old_to_new[edge_desc.source];
                        auto tgt = old_to_new[edge_desc.target];
                        result.add_edge(src, tgt, edge_desc.weight, edge_desc.type);
                    }
                }

                return result;
            }

            // Convenience overload taking a vector of vertices
            template <typename VertexProperty>
            inline Graph<VertexProperty>
            induced_subgraph(const Graph<VertexProperty> &graph,
                             const std::vector<typename Graph<VertexProperty>::VertexId> &vertices) {

                std::unordered_set<typename Graph<VertexProperty>::VertexId> vertex_set(vertices.begin(),
                                                                                        vertices.end());
                return induced_subgraph(graph, vertex_set);
            }

            // Edge subgraph - includes selected vertices and specific edges
            // Unlike induced subgraph, this allows specifying which edges to include
            template <typename VertexProperty>
            inline Graph<VertexProperty>
            edge_subgraph(const Graph<VertexProperty> &graph,
                          const std::unordered_set<typename Graph<VertexProperty>::VertexId> &vertex_set,
                          const std::unordered_set<size_t> &edge_set) {

                using VertexId = typename Graph<VertexProperty>::VertexId;
                Graph<VertexProperty> result;

                // Map old vertex IDs to new vertex IDs
                std::unordered_map<VertexId, VertexId> old_to_new;

                // Add vertices that are in the set
                for (auto v : graph.vertices()) {
                    if (vertex_set.count(v)) {
                        VertexId new_v;
                        if constexpr (std::is_same_v<VertexProperty, void>) {
                            new_v = result.add_vertex();
                        } else {
                            new_v = result.add_vertex(graph[v]);
                        }
                        old_to_new[v] = new_v;
                    }
                }

                // Add only edges that are in the edge set
                for (const auto &edge_desc : graph.edges()) {
                    if (edge_set.count(edge_desc.id) && vertex_set.count(edge_desc.source) &&
                        vertex_set.count(edge_desc.target)) {
                        auto src = old_to_new[edge_desc.source];
                        auto tgt = old_to_new[edge_desc.target];
                        result.add_edge(src, tgt, edge_desc.weight, edge_desc.type);
                    }
                }

                return result;
            }

            // ========================================================================
            // Graph Union - Combine two graphs
            // ========================================================================

            // Union of two graphs - vertices and edges from both graphs
            // Vertex IDs are NOT preserved (new IDs assigned)
            // For Graph<void>, vertices are simply combined
            // Returns mapping of old IDs to new IDs for both graphs
            struct UnionResult {
                Graph<void> graph;
                std::unordered_map<size_t, size_t> g1_mapping; // g1 old -> new
                std::unordered_map<size_t, size_t> g2_mapping; // g2 old -> new
            };

            inline UnionResult graph_union(const Graph<void> &g1, const Graph<void> &g2) {
                UnionResult result;
                Graph<void> &g = result.graph;

                // Add all vertices from g1
                for (auto v : g1.vertices()) {
                    auto new_v = g.add_vertex();
                    result.g1_mapping[v] = new_v;
                }

                // Add all vertices from g2
                for (auto v : g2.vertices()) {
                    auto new_v = g.add_vertex();
                    result.g2_mapping[v] = new_v;
                }

                // Add all edges from g1
                for (const auto &edge_desc : g1.edges()) {
                    auto src = result.g1_mapping[edge_desc.source];
                    auto tgt = result.g1_mapping[edge_desc.target];
                    g.add_edge(src, tgt, edge_desc.weight, edge_desc.type);
                }

                // Add all edges from g2
                for (const auto &edge_desc : g2.edges()) {
                    auto src = result.g2_mapping[edge_desc.source];
                    auto tgt = result.g2_mapping[edge_desc.target];
                    g.add_edge(src, tgt, edge_desc.weight, edge_desc.type);
                }

                return result;
            }

            // Disjoint union - same as union but guarantees no vertex ID conflicts
            // (This is the default behavior of graph_union)
            inline UnionResult disjoint_union(const Graph<void> &g1, const Graph<void> &g2) {
                return graph_union(g1, g2);
            }

            // ========================================================================
            // Graph Intersection - Find common structure
            // ========================================================================

            // Intersection based on vertex IDs
            // Only includes vertices that exist in BOTH graphs with the SAME ID
            // Only includes edges that exist in BOTH graphs between common vertices
            inline Graph<void> graph_intersection(const Graph<void> &g1, const Graph<void> &g2) {
                Graph<void> result;

                // Find common vertices (by ID)
                auto vertices1 = g1.vertices();
                auto vertices2 = g2.vertices();

                // Build set for quick lookup
                std::unordered_set<size_t> vertices2_set(vertices2.begin(), vertices2.end());

                std::unordered_map<size_t, size_t> old_to_new;

                // Add vertices that appear in both graphs
                for (auto v : vertices1) {
                    if (vertices2_set.count(v)) {
                        auto new_v = result.add_vertex();
                        old_to_new[v] = new_v;
                    }
                }

                // Build edge set for g2 for quick lookup
                std::unordered_set<std::pair<size_t, size_t>, PairHash> edges2_set;
                for (const auto &edge_desc : g2.edges()) {
                    edges2_set.insert({edge_desc.source, edge_desc.target});
                    // For undirected edges, also add reverse
                    if (edge_desc.type == EdgeType::Undirected) {
                        edges2_set.insert({edge_desc.target, edge_desc.source});
                    }
                }

                // Add edges that appear in both graphs
                for (const auto &edge_desc : g1.edges()) {
                    // Both vertices must be in common set
                    if (old_to_new.count(edge_desc.source) && old_to_new.count(edge_desc.target)) {
                        // Edge must exist in g2
                        if (edges2_set.count({edge_desc.source, edge_desc.target})) {
                            auto src = old_to_new[edge_desc.source];
                            auto tgt = old_to_new[edge_desc.target];
                            result.add_edge(src, tgt, edge_desc.weight, edge_desc.type);
                        }
                    }
                }

                return result;
            }

            // ========================================================================
            // Graph Complement - Invert edge structure
            // ========================================================================

            // Create complement graph (edges exist where they don't in original)
            // For simple graphs: if edge (u,v) exists, remove it; if it doesn't, add it
            // Only makes sense for undirected graphs without self-loops
            inline Graph<void> complement(const Graph<void> &graph) {
                Graph<void> result;

                // Copy all vertices
                std::unordered_map<size_t, size_t> old_to_new;
                for (auto v : graph.vertices()) {
                    auto new_v = result.add_vertex();
                    old_to_new[v] = new_v;
                }

                // Build edge set from original graph
                std::unordered_set<std::pair<size_t, size_t>, PairHash> existing_edges_set;
                for (const auto &edge_desc : graph.edges()) {
                    // Only consider undirected edges
                    if (edge_desc.type == EdgeType::Undirected) {
                        existing_edges_set.insert({std::min(edge_desc.source, edge_desc.target),
                                                   std::max(edge_desc.source, edge_desc.target)});
                    }
                }

                // Add edges for all pairs that DON'T have an edge in original
                auto vertices = graph.vertices();
                for (size_t i = 0; i < vertices.size(); ++i) {
                    for (size_t j = i + 1; j < vertices.size(); ++j) {
                        auto u = vertices[i];
                        auto v = vertices[j];
                        auto pair = std::make_pair(std::min(u, v), std::max(u, v));

                        // If edge doesn't exist in original, add it to complement
                        if (!existing_edges_set.count(pair)) {
                            result.add_edge(old_to_new[u], old_to_new[v], 1.0, EdgeType::Undirected);
                        }
                    }
                }

                return result;
            }

            // ========================================================================
            // Utility: Filter Graph by Predicate
            // ========================================================================

            // Filter vertices by predicate function
            // Creates subgraph containing only vertices where predicate returns true
            template <typename VertexProperty, typename Predicate>
            inline Graph<VertexProperty> filter_vertices(const Graph<VertexProperty> &graph, Predicate pred) {

                using VertexId = typename Graph<VertexProperty>::VertexId;
                std::unordered_set<VertexId> selected_vertices;

                // Select vertices that match predicate
                for (auto v : graph.vertices()) {
                    if (pred(v, graph)) {
                        selected_vertices.insert(v);
                    }
                }

                return induced_subgraph(graph, selected_vertices);
            }

            // Filter edges by predicate function
            // Creates subgraph with same vertices but only edges matching predicate
            template <typename VertexProperty, typename Predicate>
            inline Graph<VertexProperty> filter_edges(const Graph<VertexProperty> &graph, Predicate pred) {

                using VertexId = typename Graph<VertexProperty>::VertexId;
                // EdgeId is globally defined as size_t in graph.hpp
                using EdgeId = size_t;

                // Include all vertices - store vector first to avoid iterator issues
                auto vertices_vec = graph.vertices();
                std::unordered_set<VertexId> all_vertices(vertices_vec.begin(), vertices_vec.end());

                // Select edges that match predicate
                std::unordered_set<EdgeId> selected_edges;
                for (const auto &edge_desc : graph.edges()) {
                    if (pred(edge_desc, graph)) {
                        selected_edges.insert(edge_desc.id);
                    }
                }

                return edge_subgraph(graph, all_vertices, selected_edges);
            }

        } // namespace algorithms
    } // namespace vertex
} // namespace graphix
