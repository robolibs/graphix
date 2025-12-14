#pragma once

#include "graphix/kernel.hpp"
#include "graphix/store.hpp"
#include <vector>

namespace graphix {
    namespace vertex {

        // Forward declaration for void specialization
        template <typename VertexProperty = void> class Graph;

        // Specialization for graphs without vertex properties
        template <> class Graph<void> {
          public:
            using VertexId = Key;

            Graph() = default;

            VertexId add_vertex();
            size_t vertex_count() const;
            bool has_vertex(VertexId v) const;

          private:
            Store<int> m_vertices; // Dummy storage, just for ID generation
        };

        // General template for graphs with vertex properties
        template <typename VertexProperty> class Graph {
          public:
            using VertexId = Key;

            Graph() = default;

            // Add vertex with property
            VertexId add_vertex(const VertexProperty &prop);

            // Access vertex properties
            VertexProperty &operator[](VertexId v);
            const VertexProperty &operator[](VertexId v) const;

            // Query operations
            size_t vertex_count() const;
            bool has_vertex(VertexId v) const;

          private:
            Store<VertexProperty> m_vertices;
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

    } // namespace vertex
} // namespace graphix
