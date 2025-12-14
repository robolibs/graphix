#include "graphix/vertex/graph.hpp"
#include <stdexcept>
#include <unordered_set>

namespace graphix {
    namespace vertex {

        // Vertex operations for void specialization
        typename Graph<void>::VertexId Graph<void>::add_vertex() {
            auto id = m_vertices.add(0); // Dummy value
            return id.value();
        }

        size_t Graph<void>::vertex_count() const { return m_vertices.size(); }

        bool Graph<void>::has_vertex(VertexId v) const { return m_vertices.contains(Id<int>(v)); }

        // Edge operations for void specialization
        graphix::vertex::EdgeId Graph<void>::add_edge(VertexId u, VertexId v, double weight) {
            // Verify both vertices exist
            if (!has_vertex(u) || !has_vertex(v)) {
                throw std::invalid_argument("Cannot add edge: one or both vertices do not exist");
            }

            graphix::vertex::EdgeId edge_id = m_next_edge_id++;

            // Add edge u -> v
            m_adjacency[u].push_back({v, weight, edge_id});

            // Add edge v -> u (undirected graph)
            m_adjacency[v].push_back({u, weight, edge_id});

            m_edge_count++;
            return edge_id;
        }

        bool Graph<void>::has_edge(VertexId u, VertexId v) const {
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

        double Graph<void>::get_weight(graphix::vertex::EdgeId e) const {
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

        void Graph<void>::set_weight(graphix::vertex::EdgeId e, double weight) {
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

        size_t Graph<void>::edge_count() const { return m_edge_count; }

        // Adjacency and neighbor queries for void specialization
        std::vector<Graph<void>::VertexId> Graph<void>::neighbors(VertexId v) const {
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

        size_t Graph<void>::degree(VertexId v) const {
            auto it = m_adjacency.find(v);
            if (it != m_adjacency.end()) {
                return it->second.size();
            }
            return 0;
        }

        // Iterators for void specialization
        std::vector<Graph<void>::VertexId> Graph<void>::vertices() const {
            auto ids = m_vertices.all_ids();
            std::vector<VertexId> result;
            result.reserve(ids.size());
            for (auto id : ids) {
                result.push_back(static_cast<VertexId>(id));
            }
            return result;
        }

        std::vector<EdgeDescriptor> Graph<void>::edges() const {
            std::vector<EdgeDescriptor> result;
            result.reserve(m_edge_count);

            // Use a set to track edges we've already added (to avoid duplicates in undirected graph)
            std::unordered_set<graphix::vertex::EdgeId> seen;

            for (const auto &[source, edge_list] : m_adjacency) {
                for (const auto &edge : edge_list) {
                    // Only add each edge once (undirected graph stores each edge twice)
                    if (seen.find(edge.id) == seen.end()) {
                        result.push_back({source, edge.target, edge.weight, edge.id});
                        seen.insert(edge.id);
                    }
                }
            }

            return result;
        }

    } // namespace vertex
} // namespace graphix
