#pragma once

#include "graphix/kernel.hpp"
#include <vector>

namespace graphix {
    namespace vertex {

        class Graph {
          public:
            using NodeId = Key;
            using EdgeId = size_t;

            Graph();

            NodeId add_node();
            EdgeId add_edge(NodeId a, NodeId b);

            size_t node_count() const;
            size_t edge_count() const;

          private:
            std::vector<NodeId> m_nodes;
            std::vector<std::pair<NodeId, NodeId>> m_edges;
        };

    } // namespace vertex
} // namespace graphix
