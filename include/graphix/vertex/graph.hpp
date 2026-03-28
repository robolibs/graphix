#pragma once

#include "graphix/kernel.hpp"
#include "graphix/store.hpp"
#include <algorithm>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace graphix {
    namespace vertex {

        using EdgeId = size_t;

        // Edge type: directed or undirected
        enum class EdgeType {
            Undirected, // Bidirectional edge (default)
            Directed    // Unidirectional edge (source -> target)
        };

        // Edge descriptor for iteration
        struct EdgeDescriptor {
            size_t source;
            size_t target;
            double weight;
            EdgeId id;
            EdgeType type;
        };

        // Forward declaration for graph templates
        template <typename VertexProperty = void, typename EdgeProperty = void> class Graph;

        template <typename EdgeProperty> struct EdgePropertyHolder {
            EdgeProperty property;
        };

        template <> struct EdgePropertyHolder<void> {};

        template <typename EdgeProperty>
        using EdgePropertyParamT =
            std::conditional_t<std::is_void_v<EdgeProperty>, EdgePropertyHolder<void>, EdgeProperty>;

        // Specialization for graphs without vertex properties
        template <typename EdgeProperty> class Graph<void, EdgeProperty> {
          public:
            using VertexId = Key;
            using EdgePropertyType = EdgeProperty;

            Graph() = default;
            Graph(const Graph &other) = default;
            Graph(Graph &&other) = default;
            Graph &operator=(const Graph &other) = default;
            Graph &operator=(Graph &&other) = default;

            // Vertex operations
            inline VertexId add_vertex() {
                auto id = m_vertices.add(0); // Dummy value
                return id.value();
            }

            inline size_t vertex_count() const { return m_vertices.size(); }

            inline bool has_vertex(VertexId v) const { return m_vertices.contains(Id<int>(v)); }

            // Edge operations
            inline EdgeId add_edge(VertexId u, VertexId v, double weight = 1.0, EdgeType type = EdgeType::Undirected) {
                if constexpr (std::is_void_v<EdgeProperty>) {
                    return add_edge_impl(u, v, weight, type);
                } else {
                    static_assert(std::is_default_constructible_v<EdgeProperty>,
                                  "Graph with edge properties requires default-constructible EdgeProperty for the "
                                  "backward-compatible add_edge overload");
                    return add_edge_impl(u, v, weight, type, EdgeProperty{});
                }
            }

            template <typename EP = EdgeProperty, typename = std::enable_if_t<!std::is_void_v<EP>, int>>
            inline EdgeId add_edge(VertexId u, VertexId v, double weight, EdgeType type,
                                   const EdgePropertyParamT<EdgeProperty> &prop) {
                return add_edge_impl(u, v, weight, type, prop);
            }

            template <typename EP = EdgeProperty, typename = std::enable_if_t<!std::is_void_v<EP>, int>>
            inline EdgeId add_edge(VertexId u, VertexId v, const EdgePropertyParamT<EdgeProperty> &prop) {
                return add_edge_impl(u, v, 1.0, EdgeType::Undirected, prop);
            }

            template <typename EP = EdgeProperty, typename = std::enable_if_t<!std::is_void_v<EP>, int>>
            inline EdgePropertyParamT<EdgeProperty> &edge_property(EdgeId e) {
                for (auto &[vertex, edges] : m_adjacency) {
                    for (auto &edge : edges) {
                        if (edge.id == e) {
                            return edge.property;
                        }
                    }
                }
                throw std::invalid_argument("Edge ID not found");
            }

            template <typename EP = EdgeProperty, typename = std::enable_if_t<!std::is_void_v<EP>, int>>
            inline const EdgePropertyParamT<EdgeProperty> &edge_property(EdgeId e) const {
                for (const auto &[vertex, edges] : m_adjacency) {
                    for (const auto &edge : edges) {
                        if (edge.id == e) {
                            return edge.property;
                        }
                    }
                }
                throw std::invalid_argument("Edge ID not found");
            }

          private:
            inline EdgeId add_edge_impl(VertexId u, VertexId v, double weight, EdgeType type) {
                // Verify both vertices exist
                if (!has_vertex(u) || !has_vertex(v)) {
                    throw std::invalid_argument("Cannot add edge: one or both vertices do not exist");
                }

                EdgeId edge_id = m_next_edge_id++;

                Edge forward{};
                forward.source = u;
                forward.target = v;
                forward.weight = weight;
                forward.id = edge_id;
                forward.type = type;
                m_adjacency[u].push_back(forward);

                // For undirected edges, also add v -> u
                if (type == EdgeType::Undirected) {
                    Edge reverse{};
                    reverse.source = v;
                    reverse.target = u;
                    reverse.weight = weight;
                    reverse.id = edge_id;
                    reverse.type = type;
                    m_adjacency[v].push_back(reverse);
                }

                m_edge_count++;
                return edge_id;
            }

            inline EdgeId add_edge_impl(VertexId u, VertexId v, double weight, EdgeType type,
                                        const EdgePropertyParamT<EdgeProperty> &prop) {
                // Verify both vertices exist
                if (!has_vertex(u) || !has_vertex(v)) {
                    throw std::invalid_argument("Cannot add edge: one or both vertices do not exist");
                }

                EdgeId edge_id = m_next_edge_id++;

                Edge forward{};
                forward.source = u;
                forward.target = v;
                forward.weight = weight;
                forward.id = edge_id;
                forward.type = type;
                forward.property = prop;
                m_adjacency[u].push_back(forward);

                // For undirected edges, also add v -> u
                if (type == EdgeType::Undirected) {
                    Edge reverse{};
                    reverse.source = v;
                    reverse.target = u;
                    reverse.weight = weight;
                    reverse.id = edge_id;
                    reverse.type = type;
                    reverse.property = prop;
                    m_adjacency[v].push_back(reverse);
                }

                m_edge_count++;
                return edge_id;
            }

          public:
            inline bool has_edge(VertexId u, VertexId v) const {
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

            inline double get_weight(EdgeId e) const {
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

            inline void set_weight(EdgeId e, double weight) {
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

            inline size_t edge_count() const { return m_edge_count; }

            // Edge query functions
            inline std::optional<EdgeId> get_edge(VertexId u, VertexId v) const {
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

            inline std::pair<EdgeId, bool> edge(VertexId u, VertexId v) const {
                auto opt = get_edge(u, v);
                if (opt.has_value()) {
                    return {opt.value(), true};
                }
                return {0, false};
            }

            inline VertexId source(EdgeId e) const {
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

            inline VertexId target(EdgeId e) const {
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

            inline EdgeType get_edge_type(EdgeId e) const {
                // Search through all adjacency lists to find edge with this ID
                for (const auto &[vertex, edges] : m_adjacency) {
                    for (const auto &edge : edges) {
                        if (edge.id == e) {
                            return edge.type;
                        }
                    }
                }
                throw std::invalid_argument("Edge ID not found");
            }

            inline std::vector<EdgeId> out_edges(VertexId v) const {
                std::vector<EdgeId> result;
                auto it = m_adjacency.find(v);
                if (it != m_adjacency.end()) {
                    result.reserve(it->second.size());
                    for (const auto &edge : it->second) {
                        result.push_back(edge.id);
                    }
                }
                return result;
            }

            // Adjacency and neighbor queries
            inline std::vector<VertexId> neighbors(VertexId v) const {
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

            inline size_t degree(VertexId v) const {
                auto it = m_adjacency.find(v);
                if (it != m_adjacency.end()) {
                    return it->second.size();
                }
                return 0;
            }

            // Iterators
            inline std::vector<VertexId> vertices() const {
                auto ids = m_vertices.all_ids();
                std::vector<VertexId> result;
                result.reserve(ids.size());
                for (auto id : ids) {
                    result.push_back(static_cast<VertexId>(id));
                }
                return result;
            }

            inline std::vector<EdgeDescriptor> edges() const {
                std::vector<EdgeDescriptor> result;
                result.reserve(m_edge_count);

                // Use a set to track edges we've already added (to avoid duplicates in undirected graph)
                std::unordered_set<EdgeId> seen;

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

            // Graph modification
            inline void clear() {
                m_vertices = Store<int>();
                m_adjacency.clear();
                m_next_edge_id = 0;
                m_edge_count = 0;
            }

            inline void remove_edge(EdgeId e) {
                // Find and remove edges with this ID from both directions
                for (auto &[vertex, edges] : m_adjacency) {
                    auto it =
                        std::remove_if(edges.begin(), edges.end(), [e](const Edge &edge) { return edge.id == e; });
                    if (it != edges.end()) {
                        edges.erase(it, edges.end());
                    }
                }
                m_edge_count--;
            }

            inline void remove_edge(VertexId u, VertexId v) {
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

            inline void remove_vertex(VertexId v) {
                if (!has_vertex(v)) {
                    return;
                }

                // Remove all edges incident to this vertex
                auto it = m_adjacency.find(v);
                if (it != m_adjacency.end()) {
                    // Remove edges from this vertex to others
                    std::vector<EdgeId> edges_to_remove;
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

            // Serialization (DOT format)
            inline void save_dot(const std::string &filename) const {
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

            static inline Graph<void, EdgeProperty> load_dot(const std::string &filename) {
                std::ifstream in(filename);
                if (!in.is_open()) {
                    throw std::runtime_error("Failed to open file for reading: " + filename);
                }

                Graph<void, EdgeProperty> g;
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

          private:
            struct Edge : EdgePropertyHolder<EdgeProperty> {
                VertexId source;
                VertexId target;
                double weight;
                EdgeId id;
                EdgeType type;
            };

            Store<int> m_vertices;                                       // Dummy storage, just for ID generation
            std::unordered_map<VertexId, std::vector<Edge>> m_adjacency; // Adjacency list
            size_t m_next_edge_id = 0;                                   // Next edge ID to assign
            size_t m_edge_count = 0;                                     // Total number of unique edges
        };

        // General template for graphs with vertex properties
        template <typename VertexProperty, typename EdgeProperty> class Graph {
          public:
            using VertexId = Key;
            using EdgePropertyType = EdgeProperty;

            Graph() = default;
            Graph(const Graph &other) = default;
            Graph(Graph &&other) = default;
            Graph &operator=(const Graph &other) = default;
            Graph &operator=(Graph &&other) = default;

            // Vertex operations
            VertexId add_vertex(const VertexProperty &prop);
            VertexProperty &operator[](VertexId v);
            const VertexProperty &operator[](VertexId v) const;
            size_t vertex_count() const;
            bool has_vertex(VertexId v) const;

            // Edge operations
            EdgeId add_edge(VertexId u, VertexId v, double weight = 1.0, EdgeType type = EdgeType::Undirected);
            template <typename EP = EdgeProperty, typename = std::enable_if_t<!std::is_void_v<EP>, int>>
            EdgeId add_edge(VertexId u, VertexId v, double weight, EdgeType type,
                            const EdgePropertyParamT<EdgeProperty> &prop);

            template <typename EP = EdgeProperty, typename = std::enable_if_t<!std::is_void_v<EP>, int>>
            EdgeId add_edge(VertexId u, VertexId v, const EdgePropertyParamT<EdgeProperty> &prop);
            bool has_edge(VertexId u, VertexId v) const;
            double get_weight(EdgeId e) const;
            void set_weight(EdgeId e, double weight);
            size_t edge_count() const;

            template <typename EP = EdgeProperty, typename = std::enable_if_t<!std::is_void_v<EP>, int>>
            EdgePropertyParamT<EdgeProperty> &edge_property(EdgeId e);

            template <typename EP = EdgeProperty, typename = std::enable_if_t<!std::is_void_v<EP>, int>>
            const EdgePropertyParamT<EdgeProperty> &edge_property(EdgeId e) const;

            // Edge query functions
            std::optional<EdgeId> get_edge(VertexId u, VertexId v) const;
            std::pair<EdgeId, bool> edge(VertexId u, VertexId v) const;
            VertexId source(EdgeId e) const;
            VertexId target(EdgeId e) const;
            EdgeType get_edge_type(EdgeId e) const;
            std::vector<EdgeId> out_edges(VertexId v) const;

            // Adjacency and neighbor queries
            std::vector<VertexId> neighbors(VertexId v) const;
            size_t degree(VertexId v) const;

            // Iterators
            std::vector<VertexId> vertices() const;
            std::vector<EdgeDescriptor> edges() const;

            // Graph modification
            void clear();
            void remove_edge(EdgeId e);
            void remove_edge(VertexId u, VertexId v);
            void remove_vertex(VertexId v);

            // Serialization (DOT format)
            template <typename PropertyWriter>
            void save_dot(const std::string &filename, PropertyWriter write_prop) const;

            template <typename PropertyReader>
            static Graph<VertexProperty, EdgeProperty> load_dot(const std::string &filename, PropertyReader read_prop);

          private:
            struct Edge : EdgePropertyHolder<EdgeProperty> {
                VertexId source;
                VertexId target;
                double weight;
                EdgeId id;
                EdgeType type;
            };

            Store<VertexProperty> m_vertices;
            std::unordered_map<VertexId, std::vector<Edge>> m_adjacency; // Adjacency list
            size_t m_next_edge_id = 0;                                   // Next edge ID to assign
            size_t m_edge_count = 0;                                     // Total number of unique edges
        };

        // ============================================================================
        // Implementation
        // ============================================================================

        // Template implementation
        template <typename VertexProperty, typename EdgeProperty>
        typename Graph<VertexProperty, EdgeProperty>::VertexId
        Graph<VertexProperty, EdgeProperty>::add_vertex(const VertexProperty &prop) {
            auto id = m_vertices.add(prop);
            return id.value();
        }

        template <typename VertexProperty, typename EdgeProperty>
        VertexProperty &Graph<VertexProperty, EdgeProperty>::operator[](VertexId v) {
            return m_vertices[Id<VertexProperty>(v)];
        }

        template <typename VertexProperty, typename EdgeProperty>
        const VertexProperty &Graph<VertexProperty, EdgeProperty>::operator[](VertexId v) const {
            return m_vertices[Id<VertexProperty>(v)];
        }

        template <typename VertexProperty, typename EdgeProperty>
        size_t Graph<VertexProperty, EdgeProperty>::vertex_count() const {
            return m_vertices.size();
        }

        template <typename VertexProperty, typename EdgeProperty>
        bool Graph<VertexProperty, EdgeProperty>::has_vertex(VertexId v) const {
            return m_vertices.contains(Id<VertexProperty>(v));
        }

        // Edge operations
        template <typename VertexProperty, typename EdgeProperty>
        EdgeId Graph<VertexProperty, EdgeProperty>::add_edge(VertexId u, VertexId v, double weight, EdgeType type) {
            // Verify both vertices exist
            if (!has_vertex(u) || !has_vertex(v)) {
                throw std::invalid_argument("Cannot add edge: one or both vertices do not exist");
            }

            EdgeId edge_id = m_next_edge_id++;

            if constexpr (std::is_void_v<EdgeProperty>) {
                Edge forward{};
                forward.source = u;
                forward.target = v;
                forward.weight = weight;
                forward.id = edge_id;
                forward.type = type;
                m_adjacency[u].push_back(forward);

                // For undirected edges, also add v -> u
                if (type == EdgeType::Undirected) {
                    Edge reverse{};
                    reverse.source = v;
                    reverse.target = u;
                    reverse.weight = weight;
                    reverse.id = edge_id;
                    reverse.type = type;
                    m_adjacency[v].push_back(reverse);
                }
            } else {
                static_assert(std::is_default_constructible_v<EdgeProperty>,
                              "Graph with edge properties requires default-constructible EdgeProperty for the "
                              "backward-compatible add_edge overload");

                Edge forward{};
                forward.source = u;
                forward.target = v;
                forward.weight = weight;
                forward.id = edge_id;
                forward.type = type;
                forward.property = EdgeProperty{};
                m_adjacency[u].push_back(forward);

                // For undirected edges, also add v -> u
                if (type == EdgeType::Undirected) {
                    Edge reverse{};
                    reverse.source = v;
                    reverse.target = u;
                    reverse.weight = weight;
                    reverse.id = edge_id;
                    reverse.type = type;
                    reverse.property = EdgeProperty{};
                    m_adjacency[v].push_back(reverse);
                }
            }

            m_edge_count++;
            return edge_id;
        }

        template <typename VertexProperty, typename EdgeProperty>
        template <typename EP, typename>
        EdgeId Graph<VertexProperty, EdgeProperty>::add_edge(VertexId u, VertexId v, double weight, EdgeType type,
                                                             const EdgePropertyParamT<EdgeProperty> &prop) {
            // Verify both vertices exist
            if (!has_vertex(u) || !has_vertex(v)) {
                throw std::invalid_argument("Cannot add edge: one or both vertices do not exist");
            }

            EdgeId edge_id = m_next_edge_id++;

            Edge forward{};
            forward.source = u;
            forward.target = v;
            forward.weight = weight;
            forward.id = edge_id;
            forward.type = type;
            forward.property = prop;
            m_adjacency[u].push_back(forward);
            if (type == EdgeType::Undirected) {
                Edge reverse{};
                reverse.source = v;
                reverse.target = u;
                reverse.weight = weight;
                reverse.id = edge_id;
                reverse.type = type;
                reverse.property = prop;
                m_adjacency[v].push_back(reverse);
            }

            m_edge_count++;
            return edge_id;
        }

        template <typename VertexProperty, typename EdgeProperty>
        template <typename EP, typename>
        EdgeId Graph<VertexProperty, EdgeProperty>::add_edge(VertexId u, VertexId v,
                                                             const EdgePropertyParamT<EdgeProperty> &prop) {
            return add_edge(u, v, 1.0, EdgeType::Undirected, prop);
        }

        template <typename VertexProperty, typename EdgeProperty>
        template <typename EP, typename>
        EdgePropertyParamT<EdgeProperty> &Graph<VertexProperty, EdgeProperty>::edge_property(EdgeId e) {
            for (auto &[vertex, edges] : m_adjacency) {
                for (auto &edge : edges) {
                    if (edge.id == e) {
                        return edge.property;
                    }
                }
            }
            throw std::invalid_argument("Edge ID not found");
        }

        template <typename VertexProperty, typename EdgeProperty>
        template <typename EP, typename>
        const EdgePropertyParamT<EdgeProperty> &Graph<VertexProperty, EdgeProperty>::edge_property(EdgeId e) const {
            for (const auto &[vertex, edges] : m_adjacency) {
                for (const auto &edge : edges) {
                    if (edge.id == e) {
                        return edge.property;
                    }
                }
            }
            throw std::invalid_argument("Edge ID not found");
        }

        template <typename VertexProperty, typename EdgeProperty>
        bool Graph<VertexProperty, EdgeProperty>::has_edge(VertexId u, VertexId v) const {
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

        template <typename VertexProperty, typename EdgeProperty>
        double Graph<VertexProperty, EdgeProperty>::get_weight(EdgeId e) const {
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

        template <typename VertexProperty, typename EdgeProperty>
        void Graph<VertexProperty, EdgeProperty>::set_weight(EdgeId e, double weight) {
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

        template <typename VertexProperty, typename EdgeProperty>
        size_t Graph<VertexProperty, EdgeProperty>::edge_count() const {
            return m_edge_count;
        }

        // Edge query functions
        template <typename VertexProperty, typename EdgeProperty>
        std::optional<EdgeId> Graph<VertexProperty, EdgeProperty>::get_edge(VertexId u, VertexId v) const {
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

        template <typename VertexProperty, typename EdgeProperty>
        std::pair<EdgeId, bool> Graph<VertexProperty, EdgeProperty>::edge(VertexId u, VertexId v) const {
            auto opt = get_edge(u, v);
            if (opt.has_value()) {
                return {opt.value(), true};
            }
            return {0, false};
        }

        template <typename VertexProperty, typename EdgeProperty>
        typename Graph<VertexProperty, EdgeProperty>::VertexId
        Graph<VertexProperty, EdgeProperty>::source(EdgeId e) const {
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

        template <typename VertexProperty, typename EdgeProperty>
        typename Graph<VertexProperty, EdgeProperty>::VertexId
        Graph<VertexProperty, EdgeProperty>::target(EdgeId e) const {
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

        template <typename VertexProperty, typename EdgeProperty>
        EdgeType Graph<VertexProperty, EdgeProperty>::get_edge_type(EdgeId e) const {
            // Search through all adjacency lists to find edge with this ID
            for (const auto &[vertex, edges] : m_adjacency) {
                for (const auto &edge : edges) {
                    if (edge.id == e) {
                        return edge.type;
                    }
                }
            }
            throw std::invalid_argument("Edge ID not found");
        }

        template <typename VertexProperty, typename EdgeProperty>
        std::vector<EdgeId> Graph<VertexProperty, EdgeProperty>::out_edges(VertexId v) const {
            std::vector<EdgeId> result;
            auto it = m_adjacency.find(v);
            if (it != m_adjacency.end()) {
                result.reserve(it->second.size());
                for (const auto &edge : it->second) {
                    result.push_back(edge.id);
                }
            }
            return result;
        }

        // Adjacency and neighbor queries
        template <typename VertexProperty, typename EdgeProperty>
        std::vector<typename Graph<VertexProperty, EdgeProperty>::VertexId>
        Graph<VertexProperty, EdgeProperty>::neighbors(VertexId v) const {
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

        template <typename VertexProperty, typename EdgeProperty>
        size_t Graph<VertexProperty, EdgeProperty>::degree(VertexId v) const {
            auto it = m_adjacency.find(v);
            if (it != m_adjacency.end()) {
                return it->second.size();
            }
            return 0;
        }

        // Iterators
        template <typename VertexProperty, typename EdgeProperty>
        std::vector<typename Graph<VertexProperty, EdgeProperty>::VertexId>
        Graph<VertexProperty, EdgeProperty>::vertices() const {
            auto ids = m_vertices.all_ids();
            std::vector<VertexId> result;
            result.reserve(ids.size());
            for (auto id : ids) {
                result.push_back(static_cast<VertexId>(id));
            }
            return result;
        }

        template <typename VertexProperty, typename EdgeProperty>
        std::vector<EdgeDescriptor> Graph<VertexProperty, EdgeProperty>::edges() const {
            std::vector<EdgeDescriptor> result;
            result.reserve(m_edge_count);

            // Use a set to track edges we've already added (to avoid duplicates in undirected graph)
            std::unordered_set<EdgeId> seen;

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

        // Graph modification
        template <typename VertexProperty, typename EdgeProperty> void Graph<VertexProperty, EdgeProperty>::clear() {
            m_vertices = Store<VertexProperty>();
            m_adjacency.clear();
            m_next_edge_id = 0;
            m_edge_count = 0;
        }

        template <typename VertexProperty, typename EdgeProperty>
        void Graph<VertexProperty, EdgeProperty>::remove_edge(EdgeId e) {
            // Find and remove edges with this ID from both directions
            for (auto &[vertex, edges] : m_adjacency) {
                auto it = std::remove_if(edges.begin(), edges.end(), [e](const Edge &edge) { return edge.id == e; });
                if (it != edges.end()) {
                    edges.erase(it, edges.end());
                }
            }
            m_edge_count--;
        }

        template <typename VertexProperty, typename EdgeProperty>
        void Graph<VertexProperty, EdgeProperty>::remove_edge(VertexId u, VertexId v) {
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

        template <typename VertexProperty, typename EdgeProperty>
        void Graph<VertexProperty, EdgeProperty>::remove_vertex(VertexId v) {
            if (!has_vertex(v)) {
                return;
            }

            // Remove all edges incident to this vertex
            auto it = m_adjacency.find(v);
            if (it != m_adjacency.end()) {
                // Remove edges from this vertex to others
                std::vector<EdgeId> edges_to_remove;
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
            m_vertices.remove(Id<VertexProperty>(v));
        }

        // ============================================================================
        // Boost-style Free Functions for Compatibility
        // ============================================================================

        // Vertex count
        template <typename VertexProperty, typename EdgeProperty>
        inline size_t num_vertices(const Graph<VertexProperty, EdgeProperty> &g) {
            return g.vertex_count();
        }

        // Edge count
        template <typename VertexProperty, typename EdgeProperty>
        inline size_t num_edges(const Graph<VertexProperty, EdgeProperty> &g) {
            return g.edge_count();
        }

        // Add vertex
        template <typename VertexProperty, typename EdgeProperty>
        inline typename Graph<VertexProperty, EdgeProperty>::VertexId
        add_vertex(const VertexProperty &prop, Graph<VertexProperty, EdgeProperty> &g) {
            return g.add_vertex(prop);
        }

        template <typename EdgeProperty>
        inline typename Graph<void, EdgeProperty>::VertexId add_vertex(Graph<void, EdgeProperty> &g) {
            return g.add_vertex();
        }

        // Add edge
        template <typename VertexProperty, typename EdgeProperty>
        inline EdgeId add_edge(typename Graph<VertexProperty, EdgeProperty>::VertexId u,
                               typename Graph<VertexProperty, EdgeProperty>::VertexId v, double weight,
                               Graph<VertexProperty, EdgeProperty> &g) {
            return g.add_edge(u, v, weight);
        }

        template <typename VertexProperty, typename EdgeProperty>
        inline EdgeId add_edge(typename Graph<VertexProperty, EdgeProperty>::VertexId u,
                               typename Graph<VertexProperty, EdgeProperty>::VertexId v,
                               Graph<VertexProperty, EdgeProperty> &g) {
            return g.add_edge(u, v);
        }

        // Vertex degree
        template <typename VertexProperty, typename EdgeProperty>
        inline size_t degree(typename Graph<VertexProperty, EdgeProperty>::VertexId v,
                             const Graph<VertexProperty, EdgeProperty> &g) {
            return g.degree(v);
        }

        // Get neighbors
        template <typename VertexProperty, typename EdgeProperty>
        inline std::vector<typename Graph<VertexProperty, EdgeProperty>::VertexId>
        neighbors(typename Graph<VertexProperty, EdgeProperty>::VertexId v,
                  const Graph<VertexProperty, EdgeProperty> &g) {
            return g.neighbors(v);
        }

        // Get all vertices
        template <typename VertexProperty, typename EdgeProperty>
        inline std::vector<typename Graph<VertexProperty, EdgeProperty>::VertexId>
        vertices(const Graph<VertexProperty, EdgeProperty> &g) {
            return g.vertices();
        }

        // Get all edges
        template <typename VertexProperty, typename EdgeProperty>
        inline std::vector<EdgeDescriptor> edges(const Graph<VertexProperty, EdgeProperty> &g) {
            return g.edges();
        }

        // Clear graph
        template <typename VertexProperty, typename EdgeProperty>
        inline void clear_graph(Graph<VertexProperty, EdgeProperty> &g) {
            g.clear();
        }

        // Remove vertex
        template <typename VertexProperty, typename EdgeProperty>
        inline void remove_vertex(typename Graph<VertexProperty, EdgeProperty>::VertexId v,
                                  Graph<VertexProperty, EdgeProperty> &g) {
            g.remove_vertex(v);
        }

        // Remove edge by ID
        template <typename VertexProperty, typename EdgeProperty>
        inline void remove_edge(EdgeId e, Graph<VertexProperty, EdgeProperty> &g) {
            g.remove_edge(e);
        }

        // Remove edge by vertices
        template <typename VertexProperty, typename EdgeProperty>
        inline void remove_edge(typename Graph<VertexProperty, EdgeProperty>::VertexId u,
                                typename Graph<VertexProperty, EdgeProperty>::VertexId v,
                                Graph<VertexProperty, EdgeProperty> &g) {
            g.remove_edge(u, v);
        }

        // Edge query functions
        template <typename VertexProperty, typename EdgeProperty>
        inline std::optional<EdgeId> get_edge(typename Graph<VertexProperty, EdgeProperty>::VertexId u,
                                              typename Graph<VertexProperty, EdgeProperty>::VertexId v,
                                              const Graph<VertexProperty, EdgeProperty> &g) {
            return g.get_edge(u, v);
        }

        template <typename VertexProperty, typename EdgeProperty>
        inline std::pair<EdgeId, bool> edge(typename Graph<VertexProperty, EdgeProperty>::VertexId u,
                                            typename Graph<VertexProperty, EdgeProperty>::VertexId v,
                                            const Graph<VertexProperty, EdgeProperty> &g) {
            return g.edge(u, v);
        }

        template <typename VertexProperty, typename EdgeProperty>
        inline typename Graph<VertexProperty, EdgeProperty>::VertexId
        source(EdgeId e, const Graph<VertexProperty, EdgeProperty> &g) {
            return g.source(e);
        }

        template <typename VertexProperty, typename EdgeProperty>
        inline typename Graph<VertexProperty, EdgeProperty>::VertexId
        target(EdgeId e, const Graph<VertexProperty, EdgeProperty> &g) {
            return g.target(e);
        }

        // ============================================================================
        // Serialization Template Implementations
        // ============================================================================

        template <typename VertexProperty, typename EdgeProperty>
        template <typename PropertyWriter>
        void Graph<VertexProperty, EdgeProperty>::save_dot(const std::string &filename,
                                                           PropertyWriter write_prop) const {
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

            // Write vertices with properties
            for (auto v : vertices()) {
                std::string prop_str = write_prop(v, (*this)[v]);
                out << "  v" << v << " [label=\"" << prop_str << "\"];\n";
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

        template <typename VertexProperty, typename EdgeProperty>
        template <typename PropertyReader>
        Graph<VertexProperty, EdgeProperty> Graph<VertexProperty, EdgeProperty>::load_dot(const std::string &filename,
                                                                                          PropertyReader read_prop) {
            std::ifstream in(filename);
            if (!in.is_open()) {
                throw std::runtime_error("Failed to open file for reading: " + filename);
            }

            Graph<VertexProperty, EdgeProperty> g;
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

                // Parse vertex declaration with property: "v123 [label="property"];"
                std::regex vertex_regex(R"(v(\d+)\s*\[label=\"([^\"]*)\"\])");
                std::smatch vertex_match;
                if (std::regex_search(line, vertex_match, vertex_regex)) {
                    std::string vertex_name = "v" + vertex_match[1].str();
                    std::string prop_str = vertex_match[2].str();

                    if (id_map.find(vertex_name) == id_map.end()) {
                        VertexProperty prop = read_prop(prop_str);
                        VertexId new_id = g.add_vertex(prop);
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

                    // Vertices should have been declared already, but check
                    if (id_map.find(src_name) == id_map.end() || id_map.find(tgt_name) == id_map.end()) {
                        throw std::runtime_error("Edge references undeclared vertex");
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

                    // Vertices should have been declared already, but check
                    if (id_map.find(src_name) == id_map.end() || id_map.find(tgt_name) == id_map.end()) {
                        throw std::runtime_error("Edge references undeclared vertex");
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
