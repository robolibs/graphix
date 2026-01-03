#pragma once

#include <algorithm>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward declarations to avoid circular dependencies
namespace graphix::vertex {
    template <typename VertexProperty> class Graph;
}

namespace graphix::vertex::algorithms {

    // DFS result containing parent map and traversal order
    template <typename VertexId> struct DFSResult {
        std::unordered_map<VertexId, VertexId> parent;       // parent[v] = vertex that discovered v
        std::unordered_map<VertexId, size_t> discovery_time; // when vertex was discovered
        std::unordered_map<VertexId, size_t> finish_time;    // when vertex finished processing
        std::vector<VertexId> preorder;                      // vertices in discovery order
        std::vector<VertexId> postorder;                     // vertices in finish order
        bool target_found = false;                           // true if target was reached (when provided)
    };

    // Recursive DFS helper
    template <typename VertexProperty, typename VertexId>
    inline void dfs_visit(const graphix::vertex::Graph<VertexProperty> &graph, VertexId vertex,
                          std::unordered_set<VertexId> &visited, DFSResult<VertexId> &result, size_t &time) {

        visited.insert(vertex);
        result.discovery_time[vertex] = time++;
        result.preorder.push_back(vertex);

        // Explore neighbors
        for (const auto &neighbor : graph.neighbors(vertex)) {
            if (visited.find(neighbor) == visited.end()) {
                result.parent[neighbor] = vertex;
                dfs_visit(graph, neighbor, visited, result, time);
            }
        }

        result.finish_time[vertex] = time++;
        result.postorder.push_back(vertex);
    }

    // Depth-First Search from a source vertex (recursive)
    template <typename VertexProperty>
    inline DFSResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    dfs(const graphix::vertex::Graph<VertexProperty> &graph,
        typename graphix::vertex::Graph<VertexProperty>::VertexId source) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        DFSResult<VertexId> result;
        std::unordered_set<VertexId> visited;
        size_t time = 0;

        dfs_visit(graph, source, visited, result, time);

        return result;
    }

    // Iterative DFS implementation (alternative to recursive)
    template <typename VertexProperty>
    inline DFSResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    dfs_iterative(const graphix::vertex::Graph<VertexProperty> &graph,
                  typename graphix::vertex::Graph<VertexProperty>::VertexId source) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        DFSResult<VertexId> result;
        std::unordered_set<VertexId> visited;
        std::stack<VertexId> stack;
        size_t time = 0;

        stack.push(source);

        while (!stack.empty()) {
            VertexId current = stack.top();
            stack.pop();

            if (visited.find(current) == visited.end()) {
                visited.insert(current);
                result.discovery_time[current] = time++;
                result.preorder.push_back(current);

                // Push neighbors in reverse order so they're processed in correct order
                auto neighbors = graph.neighbors(current);
                for (auto it = neighbors.rbegin(); it != neighbors.rend(); ++it) {
                    if (visited.find(*it) == visited.end()) {
                        stack.push(*it);
                        if (result.parent.find(*it) == result.parent.end()) {
                            result.parent[*it] = current;
                        }
                    }
                }
            }
        }

        return result;
    }

    // DFS with early termination when target is found
    template <typename VertexProperty, typename VertexId>
    inline bool dfs_find_target(const graphix::vertex::Graph<VertexProperty> &graph, VertexId current, VertexId target,
                                std::unordered_set<VertexId> &visited, DFSResult<VertexId> &result, size_t &time) {

        visited.insert(current);
        result.discovery_time[current] = time++;
        result.preorder.push_back(current);

        if (current == target) {
            result.target_found = true;
            return true;
        }

        // Explore neighbors
        for (const auto &neighbor : graph.neighbors(current)) {
            if (visited.find(neighbor) == visited.end()) {
                result.parent[neighbor] = current;
                if (dfs_find_target(graph, neighbor, target, visited, result, time)) {
                    return true;
                }
            }
        }

        result.finish_time[current] = time++;
        result.postorder.push_back(current);
        return false;
    }

    // DFS with target
    template <typename VertexProperty>
    inline DFSResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    dfs(const graphix::vertex::Graph<VertexProperty> &graph,
        typename graphix::vertex::Graph<VertexProperty>::VertexId source,
        typename graphix::vertex::Graph<VertexProperty>::VertexId target) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        DFSResult<VertexId> result;
        std::unordered_set<VertexId> visited;
        size_t time = 0;

        dfs_find_target(graph, source, target, visited, result, time);

        return result;
    }

    // Reconstruct path from source to target using parent map
    template <typename VertexId>
    inline std::vector<VertexId> reconstruct_path(const DFSResult<VertexId> &result, VertexId source, VertexId target) {
        std::vector<VertexId> path;

        // Check if target was reached
        if (!result.target_found && result.discovery_time.find(target) == result.discovery_time.end()) {
            return path; // Empty path = not reachable
        }

        // Reconstruct path backwards
        VertexId current = target;
        while (current != source) {
            path.push_back(current);
            auto it = result.parent.find(current);
            if (it == result.parent.end()) {
                return {}; // Path broken
            }
            current = it->second;
        }
        path.push_back(source);

        // Reverse to get source -> target order
        std::reverse(path.begin(), path.end());
        return path;
    }

} // namespace graphix::vertex::algorithms
