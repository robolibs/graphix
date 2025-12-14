#include "graphix/vertice/graph.hpp"

namespace graphix {
    namespace vertice {

        Graph::Graph() = default;

        Graph::NodeId Graph::add_node() {
            NodeId id = m_nodes.size();
            m_nodes.push_back(id);
            return id;
        }

        Graph::EdgeId Graph::add_edge(NodeId a, NodeId b) {
            EdgeId id = m_edges.size();
            m_edges.emplace_back(a, b);
            return id;
        }

        size_t Graph::node_count() const { return m_nodes.size(); }

        size_t Graph::edge_count() const { return m_edges.size(); }

    } // namespace vertice
} // namespace graphix
