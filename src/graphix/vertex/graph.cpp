#include "graphix/vertex/graph.hpp"

namespace graphix {
    namespace vertex {

        // Implementation for void specialization
        typename Graph<void>::VertexId Graph<void>::add_vertex() {
            auto id = m_vertices.add(0); // Dummy value
            return id.value();
        }

        size_t Graph<void>::vertex_count() const { return m_vertices.size(); }

        bool Graph<void>::has_vertex(VertexId v) const { return m_vertices.contains(Id<int>(v)); }

    } // namespace vertex
} // namespace graphix
