#pragma once

#include "graphix/kernel.hpp"
#include "graphix/store.hpp"
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace graphix {
    namespace vertex {

        using EdgeId = size_t;

        // Forward declaration for void specialization
        template <typename VertexProperty = void> class Graph;

        // Specialization for graphs without vertex properties
        template <> class Graph<void> {
          public:
            using VertexId = Key;

            Graph() = default;

            // Vertex operations
            VertexId add_vertex();
            size_t vertex_count() const;
            bool has_vertex(VertexId v) const;

            // Edge operations
            EdgeId add_edge(VertexId u, VertexId v, double weight = 1.0);
            bool has_edge(VertexId u, VertexId v) const;
            double get_weight(EdgeId e) const;
            void set_weight(EdgeId e, double weight);
            size_t edge_count() const;

            // Adjacency and neighbor queries
            std::vector<VertexId> neighbors(VertexId v) const;
            size_t degree(VertexId v) const;

          private:
            struct Edge {
                VertexId target;
                double weight;
                EdgeId id;
            };

            Store<int> m_vertices;                                       // Dummy storage, just for ID generation
            std::unordered_map<VertexId, std::vector<Edge>> m_adjacency; // Adjacency list
            size_t m_next_edge_id = 0;                                   // Next edge ID to assign
            size_t m_edge_count = 0;                                     // Total number of unique edges
        };

        // General template for graphs with vertex properties
        template <typename VertexProperty> class Graph {
          public:
            using VertexId = Key;

            Graph() = default;

            // Vertex operations
            VertexId add_vertex(const VertexProperty &prop);
            VertexProperty &operator[](VertexId v);
            const VertexProperty &operator[](VertexId v) const;
            size_t vertex_count() const;
            bool has_vertex(VertexId v) const;

            // Edge operations
            EdgeId add_edge(VertexId u, VertexId v, double weight = 1.0);
            bool has_edge(VertexId u, VertexId v) const;
            double get_weight(EdgeId e) const;
            void set_weight(EdgeId e, double weight);
            size_t edge_count() const;

            // Adjacency and neighbor queries
            std::vector<VertexId> neighbors(VertexId v) const;
            size_t degree(VertexId v) const;

          private:
            struct Edge {
                VertexId target;
                double weight;
                EdgeId id;
            };

            Store<VertexProperty> m_vertices;
            std::unordered_map<VertexId, std::vector<Edge>> m_adjacency; // Adjacency list
            size_t m_next_edge_id = 0;                                   // Next edge ID to assign
            size_t m_edge_count = 0;                                     // Total number of unique edges
        };

        // ============================================================================
        // Implementation
        // ============================================================================

        // Template implementation
        template <typename VertexProperty>
        typename Graph<VertexProperty>::VertexId Graph<VertexProperty>::add_vertex(const VertexProperty &prop) {
            auto id = m_vertices.add(prop);
            return id.value();
        }

        template <typename VertexProperty> VertexProperty &Graph<VertexProperty>::operator[](VertexId v) {
            return m_vertices[Id<VertexProperty>(v)];
        }

        template <typename VertexProperty> const VertexProperty &Graph<VertexProperty>::operator[](VertexId v) const {
            return m_vertices[Id<VertexProperty>(v)];
        }

        template <typename VertexProperty> size_t Graph<VertexProperty>::vertex_count() const {
            return m_vertices.size();
        }

        template <typename VertexProperty> bool Graph<VertexProperty>::has_vertex(VertexId v) const {
            return m_vertices.contains(Id<VertexProperty>(v));
        }

        // Edge operations
        template <typename VertexProperty>
        EdgeId Graph<VertexProperty>::add_edge(VertexId u, VertexId v, double weight) {
            // Verify both vertices exist
            if (!has_vertex(u) || !has_vertex(v)) {
                throw std::invalid_argument("Cannot add edge: one or both vertices do not exist");
            }

            EdgeId edge_id = m_next_edge_id++;

            // Add edge u -> v
            m_adjacency[u].push_back({v, weight, edge_id});

            // Add edge v -> u (undirected graph)
            m_adjacency[v].push_back({u, weight, edge_id});

            m_edge_count++;
            return edge_id;
        }

        template <typename VertexProperty> bool Graph<VertexProperty>::has_edge(VertexId u, VertexId v) const {
            auto it = m_adjacency.find(u);
            if (it == m_adjacency.end()) {
                return false;
            }
            for (const auto &edge : it->second) {
                if (edge.target == v) {
                    return true;
                }
            }
            return false;
        }

        template <typename VertexProperty> double Graph<VertexProperty>::get_weight(EdgeId e) const {
            // Search through all adjacency lists to find edge with this ID
            for (const auto &[vertex, edges] : m_adjacency) {
                for (const auto &edge : edges) {
                    if (edge.id == e) {
                        return edge.weight;
                    }
                }
            }
            throw std::invalid_argument("Edge ID not found");
        }

        template <typename VertexProperty> void Graph<VertexProperty>::set_weight(EdgeId e, double weight) {
            // Update weight in both directions (undirected graph)
            bool found = false;
            for (auto &[vertex, edges] : m_adjacency) {
                for (auto &edge : edges) {
                    if (edge.id == e) {
                        edge.weight = weight;
                        found = true;
                    }
                }
            }
            if (!found) {
                throw std::invalid_argument("Edge ID not found");
            }
        }

        template <typename VertexProperty> size_t Graph<VertexProperty>::edge_count() const { return m_edge_count; }

        // Adjacency and neighbor queries
        template <typename VertexProperty>
        std::vector<typename Graph<VertexProperty>::VertexId> Graph<VertexProperty>::neighbors(VertexId v) const {
            std::vector<VertexId> result;
            auto it = m_adjacency.find(v);
            if (it != m_adjacency.end()) {
                result.reserve(it->second.size());
                for (const auto &edge : it->second) {
                    result.push_back(edge.target);
                }
            }
            return result;
        }

        template <typename VertexProperty> size_t Graph<VertexProperty>::degree(VertexId v) const {
            auto it = m_adjacency.find(v);
            if (it != m_adjacency.end()) {
                return it->second.size();
            }
            return 0;
        }

    } // namespace vertex
} // namespace graphix
