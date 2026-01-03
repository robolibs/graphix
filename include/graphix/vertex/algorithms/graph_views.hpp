#pragma once

#include "graphix/vertex/graph.hpp"
#include <functional>
#include <unordered_set>
#include <vector>

namespace graphix {
    namespace vertex {
        namespace views {

            // ========================================================================
            // ReversedGraphView - View with reversed edge directions
            // ========================================================================
            // Presents directed edges as reversed (u->v becomes v->u)
            // Undirected edges remain unchanged
            // Zero-copy: wraps original graph

            template <typename VertexProperty> class ReversedGraphView {
              public:
                using VertexId = typename Graph<VertexProperty>::VertexId;

                explicit ReversedGraphView(const Graph<VertexProperty> &graph) : m_graph(graph) {}

                // Vertex operations (delegated)
                size_t vertex_count() const { return m_graph.vertex_count(); }
                bool has_vertex(VertexId v) const { return m_graph.has_vertex(v); }
                std::vector<VertexId> vertices() const { return m_graph.vertices(); }

                // Property access (delegated)
                const VertexProperty &operator[](VertexId v) const
                requires(!std::is_same_v<VertexProperty, void>)
                {
                    return m_graph[v];
                }

                // Edge operations - reversed
                size_t edge_count() const { return m_graph.edge_count(); }

                std::vector<EdgeDescriptor> edges() const {
                    auto original = m_graph.edges();
                    std::vector<EdgeDescriptor> result;
                    result.reserve(original.size());

                    for (const auto &e : original) {
                        if (e.type == EdgeType::Directed) {
                            // Reverse directed edges
                            result.push_back({e.target, e.source, e.weight, e.id, e.type});
                        } else {
                            // Keep undirected edges as-is
                            result.push_back(e);
                        }
                    }
                    return result;
                }

                // Reversed neighbor query
                std::vector<VertexId> neighbors(VertexId v) const {
                    std::vector<VertexId> result;
                    // For reversed view, we need to find all vertices that point TO v
                    for (const auto &e : m_graph.edges()) {
                        if (e.type == EdgeType::Directed) {
                            if (e.target == v) {
                                result.push_back(e.source);
                            }
                        } else {
                            // Undirected: same as original
                            if (e.source == v) {
                                result.push_back(e.target);
                            } else if (e.target == v) {
                                result.push_back(e.source);
                            }
                        }
                    }
                    return result;
                }

                // Out-edges in reversed view = in-edges of original
                std::vector<EdgeDescriptor> out_edges(VertexId v) const {
                    std::vector<EdgeDescriptor> result;
                    for (const auto &e : m_graph.edges()) {
                        if (e.type == EdgeType::Directed) {
                            if (e.target == v) {
                                result.push_back({e.target, e.source, e.weight, e.id, e.type});
                            }
                        } else {
                            if (e.source == v || e.target == v) {
                                result.push_back(e);
                            }
                        }
                    }
                    return result;
                }

                // Access underlying graph
                const Graph<VertexProperty> &base() const { return m_graph; }

              private:
                const Graph<VertexProperty> &m_graph;
            };

            // Specialization for void
            template <> class ReversedGraphView<void> {
              public:
                using VertexId = Graph<void>::VertexId;

                explicit ReversedGraphView(const Graph<void> &graph) : m_graph(graph) {}

                size_t vertex_count() const { return m_graph.vertex_count(); }
                bool has_vertex(VertexId v) const { return m_graph.has_vertex(v); }
                std::vector<VertexId> vertices() const { return m_graph.vertices(); }
                size_t edge_count() const { return m_graph.edge_count(); }

                std::vector<EdgeDescriptor> edges() const {
                    auto original = m_graph.edges();
                    std::vector<EdgeDescriptor> result;
                    result.reserve(original.size());

                    for (const auto &e : original) {
                        if (e.type == EdgeType::Directed) {
                            result.push_back({e.target, e.source, e.weight, e.id, e.type});
                        } else {
                            result.push_back(e);
                        }
                    }
                    return result;
                }

                std::vector<VertexId> neighbors(VertexId v) const {
                    std::vector<VertexId> result;
                    for (const auto &e : m_graph.edges()) {
                        if (e.type == EdgeType::Directed) {
                            if (e.target == v) {
                                result.push_back(e.source);
                            }
                        } else {
                            if (e.source == v) {
                                result.push_back(e.target);
                            } else if (e.target == v) {
                                result.push_back(e.source);
                            }
                        }
                    }
                    return result;
                }

                std::vector<EdgeDescriptor> out_edges(VertexId v) const {
                    std::vector<EdgeDescriptor> result;
                    for (const auto &e : m_graph.edges()) {
                        if (e.type == EdgeType::Directed) {
                            if (e.target == v) {
                                result.push_back({e.target, e.source, e.weight, e.id, e.type});
                            }
                        } else {
                            if (e.source == v || e.target == v) {
                                result.push_back(e);
                            }
                        }
                    }
                    return result;
                }

                const Graph<void> &base() const { return m_graph; }

              private:
                const Graph<void> &m_graph;
            };

            // ========================================================================
            // FilteredGraphView - View with filtered vertices/edges
            // ========================================================================
            // Lazy filtering - predicates evaluated on access
            // Zero-copy: wraps original graph

            template <typename VertexProperty> class FilteredGraphView {
              public:
                using VertexId = typename Graph<VertexProperty>::VertexId;
                using VertexPredicate = std::function<bool(VertexId)>;
                using EdgePredicate = std::function<bool(const EdgeDescriptor &)>;

                FilteredGraphView(const Graph<VertexProperty> &graph, VertexPredicate vertex_pred = nullptr,
                                  EdgePredicate edge_pred = nullptr)
                    : m_graph(graph), m_vertex_pred(vertex_pred), m_edge_pred(edge_pred) {}

                // Filter only vertices
                static FilteredGraphView vertex_filter(const Graph<VertexProperty> &graph, VertexPredicate pred) {
                    return FilteredGraphView(graph, pred, nullptr);
                }

                // Filter only edges
                static FilteredGraphView edge_filter(const Graph<VertexProperty> &graph, EdgePredicate pred) {
                    return FilteredGraphView(graph, nullptr, pred);
                }

                // Vertex operations
                std::vector<VertexId> vertices() const {
                    auto all = m_graph.vertices();
                    if (!m_vertex_pred) {
                        return all;
                    }
                    std::vector<VertexId> result;
                    for (auto v : all) {
                        if (m_vertex_pred(v)) {
                            result.push_back(v);
                        }
                    }
                    return result;
                }

                size_t vertex_count() const { return vertices().size(); }

                bool has_vertex(VertexId v) const {
                    if (!m_graph.has_vertex(v)) {
                        return false;
                    }
                    return !m_vertex_pred || m_vertex_pred(v);
                }

                // Property access
                const VertexProperty &operator[](VertexId v) const
                requires(!std::is_same_v<VertexProperty, void>)
                {
                    return m_graph[v];
                }

                // Edge operations
                std::vector<EdgeDescriptor> edges() const {
                    auto all = m_graph.edges();
                    std::vector<EdgeDescriptor> result;

                    for (const auto &e : all) {
                        // Check vertex filter
                        if (m_vertex_pred) {
                            if (!m_vertex_pred(e.source) || !m_vertex_pred(e.target)) {
                                continue;
                            }
                        }
                        // Check edge filter
                        if (m_edge_pred && !m_edge_pred(e)) {
                            continue;
                        }
                        result.push_back(e);
                    }
                    return result;
                }

                size_t edge_count() const { return edges().size(); }

                std::vector<VertexId> neighbors(VertexId v) const {
                    if (!has_vertex(v)) {
                        return {};
                    }

                    std::vector<VertexId> result;
                    for (const auto &e : edges()) {
                        if (e.source == v && has_vertex(e.target)) {
                            result.push_back(e.target);
                        } else if (e.target == v && has_vertex(e.source) && e.type == EdgeType::Undirected) {
                            result.push_back(e.source);
                        }
                    }
                    return result;
                }

                std::vector<EdgeDescriptor> out_edges(VertexId v) const {
                    if (!has_vertex(v)) {
                        return {};
                    }

                    std::vector<EdgeDescriptor> result;
                    for (const auto &e : edges()) {
                        if (e.source == v || (e.target == v && e.type == EdgeType::Undirected)) {
                            result.push_back(e);
                        }
                    }
                    return result;
                }

                const Graph<VertexProperty> &base() const { return m_graph; }

              private:
                const Graph<VertexProperty> &m_graph;
                VertexPredicate m_vertex_pred;
                EdgePredicate m_edge_pred;
            };

            // Specialization for void
            template <> class FilteredGraphView<void> {
              public:
                using VertexId = Graph<void>::VertexId;
                using VertexPredicate = std::function<bool(VertexId)>;
                using EdgePredicate = std::function<bool(const EdgeDescriptor &)>;

                FilteredGraphView(const Graph<void> &graph, VertexPredicate vertex_pred = nullptr,
                                  EdgePredicate edge_pred = nullptr)
                    : m_graph(graph), m_vertex_pred(vertex_pred), m_edge_pred(edge_pred) {}

                static FilteredGraphView vertex_filter(const Graph<void> &graph, VertexPredicate pred) {
                    return FilteredGraphView(graph, pred, nullptr);
                }

                static FilteredGraphView edge_filter(const Graph<void> &graph, EdgePredicate pred) {
                    return FilteredGraphView(graph, nullptr, pred);
                }

                std::vector<VertexId> vertices() const {
                    auto all = m_graph.vertices();
                    if (!m_vertex_pred) {
                        return all;
                    }
                    std::vector<VertexId> result;
                    for (auto v : all) {
                        if (m_vertex_pred(v)) {
                            result.push_back(v);
                        }
                    }
                    return result;
                }

                size_t vertex_count() const { return vertices().size(); }

                bool has_vertex(VertexId v) const {
                    if (!m_graph.has_vertex(v)) {
                        return false;
                    }
                    return !m_vertex_pred || m_vertex_pred(v);
                }

                std::vector<EdgeDescriptor> edges() const {
                    auto all = m_graph.edges();
                    std::vector<EdgeDescriptor> result;

                    for (const auto &e : all) {
                        if (m_vertex_pred) {
                            if (!m_vertex_pred(e.source) || !m_vertex_pred(e.target)) {
                                continue;
                            }
                        }
                        if (m_edge_pred && !m_edge_pred(e)) {
                            continue;
                        }
                        result.push_back(e);
                    }
                    return result;
                }

                size_t edge_count() const { return edges().size(); }

                std::vector<VertexId> neighbors(VertexId v) const {
                    if (!has_vertex(v)) {
                        return {};
                    }

                    std::vector<VertexId> result;
                    for (const auto &e : edges()) {
                        if (e.source == v && has_vertex(e.target)) {
                            result.push_back(e.target);
                        } else if (e.target == v && has_vertex(e.source) && e.type == EdgeType::Undirected) {
                            result.push_back(e.source);
                        }
                    }
                    return result;
                }

                std::vector<EdgeDescriptor> out_edges(VertexId v) const {
                    if (!has_vertex(v)) {
                        return {};
                    }

                    std::vector<EdgeDescriptor> result;
                    for (const auto &e : edges()) {
                        if (e.source == v || (e.target == v && e.type == EdgeType::Undirected)) {
                            result.push_back(e);
                        }
                    }
                    return result;
                }

                const Graph<void> &base() const { return m_graph; }

              private:
                const Graph<void> &m_graph;
                VertexPredicate m_vertex_pred;
                EdgePredicate m_edge_pred;
            };

            // ========================================================================
            // SubgraphView - View of a vertex subset (induced subgraph view)
            // ========================================================================
            // Unlike induced_subgraph() which creates a copy, this is a lazy view
            // Keeps original vertex IDs

            template <typename VertexProperty> class SubgraphView {
              public:
                using VertexId = typename Graph<VertexProperty>::VertexId;

                SubgraphView(const Graph<VertexProperty> &graph, std::unordered_set<VertexId> vertex_set)
                    : m_graph(graph), m_vertex_set(std::move(vertex_set)) {}

                SubgraphView(const Graph<VertexProperty> &graph, const std::vector<VertexId> &vertices)
                    : m_graph(graph), m_vertex_set(vertices.begin(), vertices.end()) {}

                std::vector<VertexId> vertices() const {
                    std::vector<VertexId> result;
                    for (auto v : m_graph.vertices()) {
                        if (m_vertex_set.count(v)) {
                            result.push_back(v);
                        }
                    }
                    return result;
                }

                size_t vertex_count() const { return m_vertex_set.size(); }

                bool has_vertex(VertexId v) const { return m_vertex_set.count(v) && m_graph.has_vertex(v); }

                const VertexProperty &operator[](VertexId v) const
                requires(!std::is_same_v<VertexProperty, void>)
                {
                    return m_graph[v];
                }

                std::vector<EdgeDescriptor> edges() const {
                    std::vector<EdgeDescriptor> result;
                    for (const auto &e : m_graph.edges()) {
                        if (m_vertex_set.count(e.source) && m_vertex_set.count(e.target)) {
                            result.push_back(e);
                        }
                    }
                    return result;
                }

                size_t edge_count() const { return edges().size(); }

                std::vector<VertexId> neighbors(VertexId v) const {
                    if (!has_vertex(v)) {
                        return {};
                    }

                    std::vector<VertexId> result;
                    for (auto n : m_graph.neighbors(v)) {
                        if (m_vertex_set.count(n)) {
                            result.push_back(n);
                        }
                    }
                    return result;
                }

                std::vector<EdgeDescriptor> out_edges(VertexId v) const {
                    if (!has_vertex(v)) {
                        return {};
                    }

                    std::vector<EdgeDescriptor> result;
                    for (const auto &e : m_graph.edges()) {
                        bool from_v = (e.source == v) || (e.target == v && e.type == EdgeType::Undirected);
                        if (from_v) {
                            VertexId other = (e.source == v) ? e.target : e.source;
                            if (m_vertex_set.count(other)) {
                                result.push_back(e);
                            }
                        }
                    }
                    return result;
                }

                const Graph<VertexProperty> &base() const { return m_graph; }
                const std::unordered_set<VertexId> &vertex_set() const { return m_vertex_set; }

              private:
                const Graph<VertexProperty> &m_graph;
                std::unordered_set<VertexId> m_vertex_set;
            };

            // Specialization for void
            template <> class SubgraphView<void> {
              public:
                using VertexId = Graph<void>::VertexId;

                SubgraphView(const Graph<void> &graph, std::unordered_set<VertexId> vertex_set)
                    : m_graph(graph), m_vertex_set(std::move(vertex_set)) {}

                SubgraphView(const Graph<void> &graph, const std::vector<VertexId> &vertices)
                    : m_graph(graph), m_vertex_set(vertices.begin(), vertices.end()) {}

                std::vector<VertexId> vertices() const {
                    std::vector<VertexId> result;
                    for (auto v : m_graph.vertices()) {
                        if (m_vertex_set.count(v)) {
                            result.push_back(v);
                        }
                    }
                    return result;
                }

                size_t vertex_count() const { return m_vertex_set.size(); }
                bool has_vertex(VertexId v) const { return m_vertex_set.count(v) && m_graph.has_vertex(v); }

                std::vector<EdgeDescriptor> edges() const {
                    std::vector<EdgeDescriptor> result;
                    for (const auto &e : m_graph.edges()) {
                        if (m_vertex_set.count(e.source) && m_vertex_set.count(e.target)) {
                            result.push_back(e);
                        }
                    }
                    return result;
                }

                size_t edge_count() const { return edges().size(); }

                std::vector<VertexId> neighbors(VertexId v) const {
                    if (!has_vertex(v)) {
                        return {};
                    }

                    std::vector<VertexId> result;
                    for (auto n : m_graph.neighbors(v)) {
                        if (m_vertex_set.count(n)) {
                            result.push_back(n);
                        }
                    }
                    return result;
                }

                std::vector<EdgeDescriptor> out_edges(VertexId v) const {
                    if (!has_vertex(v)) {
                        return {};
                    }

                    std::vector<EdgeDescriptor> result;
                    for (const auto &e : m_graph.edges()) {
                        bool from_v = (e.source == v) || (e.target == v && e.type == EdgeType::Undirected);
                        if (from_v) {
                            VertexId other = (e.source == v) ? e.target : e.source;
                            if (m_vertex_set.count(other)) {
                                result.push_back(e);
                            }
                        }
                    }
                    return result;
                }

                const Graph<void> &base() const { return m_graph; }
                const std::unordered_set<VertexId> &vertex_set() const { return m_vertex_set; }

              private:
                const Graph<void> &m_graph;
                std::unordered_set<VertexId> m_vertex_set;
            };

            // ========================================================================
            // Helper functions for creating views
            // ========================================================================

            template <typename VertexProperty>
            inline ReversedGraphView<VertexProperty> reversed(const Graph<VertexProperty> &graph) {
                return ReversedGraphView<VertexProperty>(graph);
            }

            inline ReversedGraphView<void> reversed(const Graph<void> &graph) { return ReversedGraphView<void>(graph); }

            template <typename VertexProperty, typename VertexPred>
            inline FilteredGraphView<VertexProperty> filter_vertices_view(const Graph<VertexProperty> &graph,
                                                                          VertexPred pred) {
                return FilteredGraphView<VertexProperty>::vertex_filter(graph, pred);
            }

            template <typename VertexPred>
            inline FilteredGraphView<void> filter_vertices_view(const Graph<void> &graph, VertexPred pred) {
                return FilteredGraphView<void>::vertex_filter(graph, pred);
            }

            template <typename VertexProperty, typename EdgePred>
            inline FilteredGraphView<VertexProperty> filter_edges_view(const Graph<VertexProperty> &graph,
                                                                       EdgePred pred) {
                return FilteredGraphView<VertexProperty>::edge_filter(graph, pred);
            }

            template <typename EdgePred>
            inline FilteredGraphView<void> filter_edges_view(const Graph<void> &graph, EdgePred pred) {
                return FilteredGraphView<void>::edge_filter(graph, pred);
            }

            template <typename VertexProperty>
            inline SubgraphView<VertexProperty>
            subgraph_view(const Graph<VertexProperty> &graph,
                          const std::unordered_set<typename Graph<VertexProperty>::VertexId> &vertices) {
                return SubgraphView<VertexProperty>(graph, vertices);
            }

            inline SubgraphView<void> subgraph_view(const Graph<void> &graph,
                                                    const std::unordered_set<Graph<void>::VertexId> &vertices) {
                return SubgraphView<void>(graph, vertices);
            }

            template <typename VertexProperty>
            inline SubgraphView<VertexProperty>
            subgraph_view(const Graph<VertexProperty> &graph,
                          const std::vector<typename Graph<VertexProperty>::VertexId> &vertices) {
                return SubgraphView<VertexProperty>(graph, vertices);
            }

            inline SubgraphView<void> subgraph_view(const Graph<void> &graph,
                                                    const std::vector<Graph<void>::VertexId> &vertices) {
                return SubgraphView<void>(graph, vertices);
            }

        } // namespace views
    } // namespace vertex
} // namespace graphix
