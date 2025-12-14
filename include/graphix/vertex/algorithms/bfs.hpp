#pragma once

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward declarations to avoid circular dependencies
namespace graphix::vertex {
    template <typename VertexProperty> class Graph;
}

namespace graphix::vertex::algorithms {

    // BFS result containing parent map and distances from source
    template <typename VertexId> struct BFSResult {
        std::unordered_map<VertexId, VertexId> parent; // parent[v] = vertex that discovered v
        std::unordered_map<VertexId, size_t> distance; // distance[v] = distance from source to v
        std::vector<VertexId> discovery_order;         // order vertices were discovered
        bool target_found = false;                     // true if target was reached (when provided)
    };

    // Breadth-First Search from a source vertex
    // Returns parent map, distances, and discovery order
    template <typename VertexProperty>
    inline BFSResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    bfs(const graphix::vertex::Graph<VertexProperty> &graph,
        typename graphix::vertex::Graph<VertexProperty>::VertexId source) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        BFSResult<VertexId> result;
        std::unordered_set<VertexId> visited;
        std::queue<VertexId> queue;

        // Initialize
        queue.push(source);
        visited.insert(source);
        result.distance[source] = 0;
        result.discovery_order.push_back(source);

        while (!queue.empty()) {
            VertexId current = queue.front();
            queue.pop();

            // Explore neighbors
            for (const auto &neighbor : graph.neighbors(current)) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    queue.push(neighbor);

                    result.parent[neighbor] = current;
                    result.distance[neighbor] = result.distance[current] + 1;
                    result.discovery_order.push_back(neighbor);
                }
            }
        }

        return result;
    }

    // BFS with early termination when target is found
    // Returns path from source to target if reachable
    template <typename VertexProperty>
    inline BFSResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    bfs(const graphix::vertex::Graph<VertexProperty> &graph,
        typename graphix::vertex::Graph<VertexProperty>::VertexId source,
        typename graphix::vertex::Graph<VertexProperty>::VertexId target) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        BFSResult<VertexId> result;
        std::unordered_set<VertexId> visited;
        std::queue<VertexId> queue;

        // Initialize
        queue.push(source);
        visited.insert(source);
        result.distance[source] = 0;
        result.discovery_order.push_back(source);

        while (!queue.empty()) {
            VertexId current = queue.front();
            queue.pop();

            // Early exit if target found
            if (current == target) {
                result.target_found = true;
                return result;
            }

            // Explore neighbors
            for (const auto &neighbor : graph.neighbors(current)) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    queue.push(neighbor);

                    result.parent[neighbor] = current;
                    result.distance[neighbor] = result.distance[current] + 1;
                    result.discovery_order.push_back(neighbor);
                }
            }
        }

        return result;
    }

    // Reconstruct path from source to target using parent map
    template <typename VertexId>
    inline std::vector<VertexId> reconstruct_path(const BFSResult<VertexId> &result, VertexId source, VertexId target) {
        std::vector<VertexId> path;

        // Check if target was reached
        if (result.distance.find(target) == result.distance.end()) {
            return path; // Empty path = not reachable
        }

        // Reconstruct path backwards
        VertexId current = target;
        while (current != source) {
            path.push_back(current);
            auto it = result.parent.find(current);
            if (it == result.parent.end()) {
                return {}; // Path broken (shouldn't happen if distance check passed)
            }
            current = it->second;
        }
        path.push_back(source);

        // Reverse to get source -> target order
        std::reverse(path.begin(), path.end());
        return path;
    }

} // namespace graphix::vertex::algorithms
