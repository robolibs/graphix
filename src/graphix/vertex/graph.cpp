#include "graphix/vertex/graph.hpp"
#include <algorithm>
#include <fstream>
#include <map>
#include <regex>
#include <sstream>
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
        graphix::vertex::EdgeId Graph<void>::add_edge(VertexId u, VertexId v, double weight, EdgeType type) {
            // Verify both vertices exist
            if (!has_vertex(u) || !has_vertex(v)) {
                throw std::invalid_argument("Cannot add edge: one or both vertices do not exist");
            }

            graphix::vertex::EdgeId edge_id = m_next_edge_id++;

            // Add edge u -> v
            m_adjacency[u].push_back({u, v, weight, edge_id, type});

            // For undirected edges, also add v -> u
            if (type == EdgeType::Undirected) {
                m_adjacency[v].push_back({v, u, weight, edge_id, type});
            }

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

        // Edge query functions for void specialization
        std::optional<graphix::vertex::EdgeId> Graph<void>::get_edge(VertexId u, VertexId v) const {
            auto it = m_adjacency.find(u);
            if (it == m_adjacency.end()) {
                return std::nullopt;
            }
            for (const auto &edge : it->second) {
                if (edge.target == v) {
                    return edge.id;
                }
            }
            return std::nullopt;
        }

        std::pair<graphix::vertex::EdgeId, bool> Graph<void>::edge(VertexId u, VertexId v) const {
            auto opt = get_edge(u, v);
            if (opt.has_value()) {
                return {opt.value(), true};
            }
            return {0, false};
        }

        Graph<void>::VertexId Graph<void>::source(graphix::vertex::EdgeId e) const {
            // Search through all adjacency lists to find edge with this ID
            // Return the canonical direction (smaller vertex as source)
            VertexId found_src = 0, found_tgt = 0;
            bool found = false;
            for (const auto &[vertex, edges] : m_adjacency) {
                for (const auto &edge : edges) {
                    if (edge.id == e) {
                        if (!found || edge.source < found_src) {
                            found_src = edge.source;
                            found_tgt = edge.target;
                            found = true;
                        }
                    }
                }
            }
            if (!found) {
                throw std::invalid_argument("Edge ID not found");
            }
            return found_src;
        }

        Graph<void>::VertexId Graph<void>::target(graphix::vertex::EdgeId e) const {
            // Search through all adjacency lists to find edge with this ID
            // Return the canonical direction (smaller vertex as source)
            VertexId found_src = 0, found_tgt = 0;
            bool found = false;
            for (const auto &[vertex, edges] : m_adjacency) {
                for (const auto &edge : edges) {
                    if (edge.id == e) {
                        if (!found || edge.source < found_src) {
                            found_src = edge.source;
                            found_tgt = edge.target;
                            found = true;
                        }
                    }
                }
            }
            if (!found) {
                throw std::invalid_argument("Edge ID not found");
            }
            return found_tgt;
        }

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
                    // Only add each edge once (undirected edges are stored twice, directed only once)
                    if (seen.find(edge.id) == seen.end()) {
                        result.push_back({source, edge.target, edge.weight, edge.id, edge.type});
                        seen.insert(edge.id);
                    }
                }
            }

            return result;
        }

        // Graph modification for void specialization
        void Graph<void>::clear() {
            m_vertices = Store<int>();
            m_adjacency.clear();
            m_next_edge_id = 0;
            m_edge_count = 0;
        }

        void Graph<void>::remove_edge(graphix::vertex::EdgeId e) {
            // Find and remove edges with this ID from both directions
            for (auto &[vertex, edges] : m_adjacency) {
                auto it = std::remove_if(edges.begin(), edges.end(), [e](const Edge &edge) { return edge.id == e; });
                if (it != edges.end()) {
                    edges.erase(it, edges.end());
                }
            }
            m_edge_count--;
        }

        void Graph<void>::remove_edge(VertexId u, VertexId v) {
            // Find edge ID first
            auto it_u = m_adjacency.find(u);
            if (it_u != m_adjacency.end()) {
                for (const auto &edge : it_u->second) {
                    if (edge.target == v) {
                        remove_edge(edge.id);
                        return;
                    }
                }
            }
        }

        void Graph<void>::remove_vertex(VertexId v) {
            if (!has_vertex(v)) {
                return;
            }

            // Remove all edges incident to this vertex
            auto it = m_adjacency.find(v);
            if (it != m_adjacency.end()) {
                // Remove edges from this vertex to others
                std::vector<graphix::vertex::EdgeId> edges_to_remove;
                for (const auto &edge : it->second) {
                    edges_to_remove.push_back(edge.id);
                }
                m_adjacency.erase(it);

                // Remove edges from other vertices to this one
                for (auto edge_id : edges_to_remove) {
                    for (auto &[vertex, edges] : m_adjacency) {
                        auto edge_it = std::remove_if(edges.begin(), edges.end(),
                                                      [edge_id](const Edge &e) { return e.id == edge_id; });
                        if (edge_it != edges.end()) {
                            edges.erase(edge_it, edges.end());
                        }
                    }
                    m_edge_count--;
                }
            }

            // Remove vertex from storage
            m_vertices.remove(Id<int>(v));
        }

        // Serialization for void specialization
        void Graph<void>::save_dot(const std::string &filename) const {
            std::ofstream out(filename);
            if (!out.is_open()) {
                throw std::runtime_error("Failed to open file for writing: " + filename);
            }

            // Check if we have any directed edges
            bool has_directed = false;
            for (const auto &edge_desc : edges()) {
                if (edge_desc.type == EdgeType::Directed) {
                    has_directed = true;
                    break;
                }
            }

            // Header - use digraph if ANY directed edges exist
            out << (has_directed ? "digraph" : "graph") << " G {\n";

            // Write vertices
            for (auto v : vertices()) {
                out << "  v" << v << ";\n";
            }

            // Write edges (edges() already deduplicates for us)
            for (const auto &edge : edges()) {
                if (edge.type == EdgeType::Directed) {
                    out << "  v" << edge.source << " -> v" << edge.target;
                } else {
                    if (has_directed) {
                        // In a digraph, undirected edges need special marking
                        out << "  v" << edge.source << " -> v" << edge.target;
                    } else {
                        // In pure undirected graph, use --
                        out << "  v" << edge.source << " -- v" << edge.target;
                    }
                }
                out << " [weight=" << edge.weight;
                if (has_directed && edge.type == EdgeType::Undirected) {
                    out << ",dir=none"; // Mark undirected edges in mixed graphs
                }
                out << "];\n";
            }

            out << "}\n";
            out.close();
        }

        Graph<void> Graph<void>::load_dot(const std::string &filename) {
            std::ifstream in(filename);
            if (!in.is_open()) {
                throw std::runtime_error("Failed to open file for reading: " + filename);
            }

            Graph<void> g;
            std::map<std::string, VertexId> id_map;
            bool is_directed = false;

            std::string line;
            while (std::getline(in, line)) {
                // Remove comments
                size_t comment_pos = line.find("//");
                if (comment_pos != std::string::npos) {
                    line = line.substr(0, comment_pos);
                }

                // Trim whitespace
                line.erase(0, line.find_first_not_of(" \t\r\n"));
                line.erase(line.find_last_not_of(" \t\r\n") + 1);

                // Check for graph type declaration
                if (line.find("digraph") != std::string::npos) {
                    is_directed = true;
                    continue;
                } else if (line.find("graph") != std::string::npos && line.find("digraph") == std::string::npos) {
                    is_directed = false;
                    continue;
                }

                // Skip empty lines and closing brace
                if (line.empty() || line == "}") {
                    continue;
                }

                // Parse vertex declaration: "v123;"
                std::regex vertex_regex(R"(v(\d+);)");
                std::smatch vertex_match;
                if (std::regex_search(line, vertex_match, vertex_regex)) {
                    std::string vertex_name = "v" + vertex_match[1].str();
                    if (id_map.find(vertex_name) == id_map.end()) {
                        VertexId new_id = g.add_vertex();
                        id_map[vertex_name] = new_id;
                    }
                    continue;
                }

                // Parse directed edge: "v0 -> v1 [weight=1.5];" or "v0 -> v1 [weight=1.5,dir=none];"
                std::regex directed_edge_regex(R"(v(\d+)\s*->\s*v(\d+)\s*\[([^\]]*)\])");
                std::smatch edge_match;
                if (std::regex_search(line, edge_match, directed_edge_regex)) {
                    std::string src_name = "v" + edge_match[1].str();
                    std::string tgt_name = "v" + edge_match[2].str();
                    std::string attrs = edge_match[3].str();

                    // Create vertices if they don't exist
                    if (id_map.find(src_name) == id_map.end()) {
                        id_map[src_name] = g.add_vertex();
                    }
                    if (id_map.find(tgt_name) == id_map.end()) {
                        id_map[tgt_name] = g.add_vertex();
                    }

                    // Parse weight
                    double weight = 1.0;
                    std::regex weight_regex(R"(weight=([0-9.]+))");
                    std::smatch weight_match;
                    if (std::regex_search(attrs, weight_match, weight_regex)) {
                        weight = std::stod(weight_match[1].str());
                    }

                    // Check for dir=none (undirected edge in digraph)
                    bool is_undirected = (attrs.find("dir=none") != std::string::npos);

                    EdgeType edge_type = is_undirected ? EdgeType::Undirected : EdgeType::Directed;
                    g.add_edge(id_map[src_name], id_map[tgt_name], weight, edge_type);
                    continue;
                }

                // Parse undirected edge: "v0 -- v1 [weight=1.5];"
                std::regex undirected_edge_regex(R"(v(\d+)\s*--\s*v(\d+)\s*\[([^\]]*)\])");
                if (std::regex_search(line, edge_match, undirected_edge_regex)) {
                    std::string src_name = "v" + edge_match[1].str();
                    std::string tgt_name = "v" + edge_match[2].str();
                    std::string attrs = edge_match[3].str();

                    // Create vertices if they don't exist
                    if (id_map.find(src_name) == id_map.end()) {
                        id_map[src_name] = g.add_vertex();
                    }
                    if (id_map.find(tgt_name) == id_map.end()) {
                        id_map[tgt_name] = g.add_vertex();
                    }

                    // Parse weight
                    double weight = 1.0;
                    std::regex weight_regex(R"(weight=([0-9.]+))");
                    std::smatch weight_match;
                    if (std::regex_search(attrs, weight_match, weight_regex)) {
                        weight = std::stod(weight_match[1].str());
                    }

                    g.add_edge(id_map[src_name], id_map[tgt_name], weight, EdgeType::Undirected);
                    continue;
                }
            }

            in.close();
            return g;
        }

    } // namespace vertex
} // namespace graphix
