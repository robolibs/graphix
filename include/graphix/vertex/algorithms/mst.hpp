#pragma once

#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward declarations
namespace graphix::vertex {
    template <typename VertexProperty> class Graph;
}

namespace graphix::vertex::algorithms {

    // MST result
    template <typename VertexId> struct MSTResult {
        std::vector<size_t> edges;                     // Edge IDs in the MST
        double total_weight = 0.0;                     // Total weight of MST
        bool is_spanning_tree = false;                 // True if all vertices are connected
        std::unordered_map<VertexId, VertexId> parent; // Parent map (for verification)
    };

    // Priority queue element for Prim's algorithm
    template <typename VertexId> struct PrimEdge {
        VertexId vertex;
        size_t edge_id;
        double weight;

        bool operator>(const PrimEdge &other) const { return weight > other.weight; }
    };

    // Prim's algorithm for Minimum Spanning Tree
    template <typename VertexProperty>
    MSTResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    prim_mst(const graphix::vertex::Graph<VertexProperty> &graph) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        MSTResult<VertexId> result;

        // Get all vertices
        auto vertices = graph.vertices();
        if (vertices.empty()) {
            result.is_spanning_tree = true;
            return result;
        }

        std::unordered_set<VertexId> in_mst;
        std::priority_queue<PrimEdge<VertexId>, std::vector<PrimEdge<VertexId>>, std::greater<PrimEdge<VertexId>>> pq;

        // Start from first vertex
        VertexId start = vertices[0];
        in_mst.insert(start);

        // Add all edges from start vertex to priority queue
        auto all_edges = graph.edges();
        for (const auto &edge_desc : all_edges) {
            if (edge_desc.source == start || edge_desc.target == start) {
                VertexId neighbor = (edge_desc.source == start) ? edge_desc.target : edge_desc.source;
                pq.push({neighbor, edge_desc.id, edge_desc.weight});
            }
        }

        // Prim's algorithm main loop
        while (!pq.empty()) {
            auto [vertex, edge_id, weight] = pq.top();
            pq.pop();

            // Skip if vertex already in MST
            if (in_mst.find(vertex) != in_mst.end()) {
                continue;
            }

            // Add vertex to MST
            in_mst.insert(vertex);
            result.edges.push_back(edge_id);
            result.total_weight += weight;

            // Add parent for this vertex
            VertexId parent_vertex = (graph.source(edge_id) == vertex) ? graph.target(edge_id) : graph.source(edge_id);
            result.parent[vertex] = parent_vertex;

            // Add all edges from new vertex to priority queue
            for (const auto &edge_desc : all_edges) {
                if (edge_desc.source == vertex || edge_desc.target == vertex) {
                    VertexId neighbor = (edge_desc.source == vertex) ? edge_desc.target : edge_desc.source;
                    if (in_mst.find(neighbor) == in_mst.end()) {
                        pq.push({neighbor, edge_desc.id, edge_desc.weight});
                    }
                }
            }
        }

        // Check if we got a spanning tree (all vertices covered)
        result.is_spanning_tree = (in_mst.size() == vertices.size());

        return result;
    }

    // Prim's MST starting from a specific vertex
    template <typename VertexProperty>
    MSTResult<typename graphix::vertex::Graph<VertexProperty>::VertexId>
    prim_mst(const graphix::vertex::Graph<VertexProperty> &graph,
             typename graphix::vertex::Graph<VertexProperty>::VertexId start) {

        using VertexId = typename graphix::vertex::Graph<VertexProperty>::VertexId;

        MSTResult<VertexId> result;

        std::unordered_set<VertexId> in_mst;
        std::priority_queue<PrimEdge<VertexId>, std::vector<PrimEdge<VertexId>>, std::greater<PrimEdge<VertexId>>> pq;

        // Start from specified vertex
        in_mst.insert(start);

        // Add all edges from start vertex
        auto all_edges = graph.edges();
        for (const auto &edge_desc : all_edges) {
            if (edge_desc.source == start || edge_desc.target == start) {
                VertexId neighbor = (edge_desc.source == start) ? edge_desc.target : edge_desc.source;
                pq.push({neighbor, edge_desc.id, edge_desc.weight});
            }
        }

        // Prim's algorithm main loop
        while (!pq.empty()) {
            auto [vertex, edge_id, weight] = pq.top();
            pq.pop();

            if (in_mst.find(vertex) != in_mst.end()) {
                continue;
            }

            in_mst.insert(vertex);
            result.edges.push_back(edge_id);
            result.total_weight += weight;

            VertexId parent_vertex = (graph.source(edge_id) == vertex) ? graph.target(edge_id) : graph.source(edge_id);
            result.parent[vertex] = parent_vertex;

            for (const auto &edge_desc : all_edges) {
                if (edge_desc.source == vertex || edge_desc.target == vertex) {
                    VertexId neighbor = (edge_desc.source == vertex) ? edge_desc.target : edge_desc.source;
                    if (in_mst.find(neighbor) == in_mst.end()) {
                        pq.push({neighbor, edge_desc.id, edge_desc.weight});
                    }
                }
            }
        }

        // Check if all vertices reachable from start are in MST
        auto all_vertices = graph.vertices();
        result.is_spanning_tree = (in_mst.size() == all_vertices.size());

        return result;
    }

} // namespace graphix::vertex::algorithms
