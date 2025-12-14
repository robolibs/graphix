#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward declarations
namespace graphix::vertex {
    template <typename VertexProperty> class Graph;
}

namespace graphix::vertex::algorithms {

    // Connected components result
    template <typename VertexId> struct ComponentsResult {
        std::unordered_map<VertexId, size_t> component_id; // Maps vertex to its component ID
        std::vector<std::vector<VertexId>> components;     // List of components (each is a list of vertices)
        size_t num_components = 0;                         // Total number of components
    };

    // Helper: DFS to mark all vertices in a component
    template <typename VertexProperty, typename VertexId>
    inline void mark_component(const graphix::vertex::Graph<VertexProperty> &graph, VertexId vertex,
                               std::unordered_set<VertexId> &visited, std::vector<VertexId> &component) {
        visited.insert(vertex);
        component.push_back(vertex);

        for (const auto &neighbor : graph.neighbors(vertex)) {
            if (visited.find(neighbor) == visited.end()) {
                mark_component(graph, neighbor, visited, component);
            }
        }
    }

    // Find all connected components in an undirected graph
    template <typename VertexProperty>
    inline ComponentsResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    connected_components(const graphix::vertex::Graph<VertexProperty> &graph) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        ComponentsResult<VertexId> result;
        std::unordered_set<VertexId> visited;

        // Iterate through all vertices
        for (const auto &vertex : graph.vertices()) {
            if (visited.find(vertex) == visited.end()) {
                // Found a new component
                std::vector<VertexId> component;
                mark_component(graph, vertex, visited, component);

                // Assign component ID to all vertices in this component
                size_t component_id = result.num_components;
                for (const auto &v : component) {
                    result.component_id[v] = component_id;
                }

                result.components.push_back(std::move(component));
                result.num_components++;
            }
        }

        return result;
    }

    // Check if the graph is connected (only one component)
    template <typename VertexProperty> inline bool is_connected(const graphix::vertex::Graph<VertexProperty> &graph) {
        auto result = connected_components(graph);
        return result.num_components <= 1;
    }

    // Get the size of the largest connected component
    template <typename VertexProperty>
    inline size_t largest_component_size(const graphix::vertex::Graph<VertexProperty> &graph) {
        auto result = connected_components(graph);

        if (result.components.empty()) {
            return 0;
        }

        size_t max_size = 0;
        for (const auto &component : result.components) {
            max_size = std::max(max_size, component.size());
        }
        return max_size;
    }

    // Check if two vertices are in the same connected component
    template <typename VertexProperty>
    inline bool same_component(const graphix::vertex::Graph<VertexProperty> &graph,
                               typename graphix::vertex::Graph<VertexProperty>::VertexId u,
                               typename graphix::vertex::Graph<VertexProperty>::VertexId v) {
        auto result = connected_components(graph);

        auto u_it = result.component_id.find(u);
        auto v_it = result.component_id.find(v);

        if (u_it == result.component_id.end() || v_it == result.component_id.end()) {
            return false;
        }

        return u_it->second == v_it->second;
    }

} // namespace graphix::vertex::algorithms
