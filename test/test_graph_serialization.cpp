#include "graphix/vertex/graph.hpp"
#include <cstdio>
#include <doctest/doctest.h>
#include <fstream>
#include <sstream>

using namespace graphix::vertex;

// Simple property type for testing
struct Point2D {
    double x;
    double y;

    Point2D() : x(0), y(0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}

    bool operator==(const Point2D &other) const { return x == other.x && y == other.y; }
};

TEST_SUITE("Graph Serialization - DOT Format") {

    TEST_CASE("DOT Serialization - Empty graph") {
        Graph<void> g1;
        g1.save_dot("/tmp/test_empty.dot");

        auto g2 = Graph<void>::load_dot("/tmp/test_empty.dot");

        CHECK(g2.vertex_count() == 0);
        CHECK(g2.edge_count() == 0);

        std::remove("/tmp/test_empty.dot");
    }

    TEST_CASE("DOT Serialization - Single vertex") {
        Graph<void> g1;
        g1.add_vertex();
        g1.save_dot("/tmp/test_single.dot");

        auto g2 = Graph<void>::load_dot("/tmp/test_single.dot");

        CHECK(g2.vertex_count() == 1);
        CHECK(g2.edge_count() == 0);

        std::remove("/tmp/test_single.dot");
    }

    TEST_CASE("DOT Serialization - Simple undirected graph") {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        auto v2 = g1.add_vertex();
        g1.add_edge(v0, v1, 1.5);
        g1.add_edge(v1, v2, 2.5);

        g1.save_dot("/tmp/test_undirected.dot");
        auto g2 = Graph<void>::load_dot("/tmp/test_undirected.dot");

        CHECK(g2.vertex_count() == 3);
        CHECK(g2.edge_count() == 2);

        // Verify edges exist
        CHECK(g2.has_edge(v0, v1));
        CHECK(g2.has_edge(v1, v0)); // Undirected
        CHECK(g2.has_edge(v1, v2));
        CHECK(g2.has_edge(v2, v1)); // Undirected

        // Verify weights
        auto e1 = g2.get_edge(v0, v1);
        REQUIRE(e1.has_value());
        CHECK(g2.get_weight(e1.value()) == doctest::Approx(1.5));

        auto e2 = g2.get_edge(v1, v2);
        REQUIRE(e2.has_value());
        CHECK(g2.get_weight(e2.value()) == doctest::Approx(2.5));

        std::remove("/tmp/test_undirected.dot");
    }

    TEST_CASE("DOT Serialization - Directed graph") {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        auto v2 = g1.add_vertex();
        g1.add_edge(v0, v1, 1.5, EdgeType::Directed);
        g1.add_edge(v1, v2, 2.5, EdgeType::Directed);

        g1.save_dot("/tmp/test_directed.dot");
        auto g2 = Graph<void>::load_dot("/tmp/test_directed.dot");

        CHECK(g2.vertex_count() == 3);
        CHECK(g2.edge_count() == 2);

        // Verify directed edges
        CHECK(g2.has_edge(v0, v1));
        CHECK_FALSE(g2.has_edge(v1, v0)); // Directed, not bidirectional

        CHECK(g2.has_edge(v1, v2));
        CHECK_FALSE(g2.has_edge(v2, v1)); // Directed

        std::remove("/tmp/test_directed.dot");
    }

    TEST_CASE("DOT Serialization - Mixed graph (directed and undirected)") {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        auto v2 = g1.add_vertex();
        g1.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g1.add_edge(v1, v2, 2.0, EdgeType::Directed);

        g1.save_dot("/tmp/test_mixed.dot");
        auto g2 = Graph<void>::load_dot("/tmp/test_mixed.dot");

        CHECK(g2.vertex_count() == 3);
        CHECK(g2.edge_count() == 2);

        // Verify undirected edge
        CHECK(g2.has_edge(v0, v1));
        CHECK(g2.has_edge(v1, v0));

        // Verify directed edge
        CHECK(g2.has_edge(v1, v2));
        CHECK_FALSE(g2.has_edge(v2, v1));

        std::remove("/tmp/test_mixed.dot");
    }

    TEST_CASE("DOT Serialization - Multiple components") {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        auto v2 = g1.add_vertex();
        auto v3 = g1.add_vertex();

        // Component 1: v0 -- v1
        g1.add_edge(v0, v1, 1.0);

        // Component 2: v2 -- v3
        g1.add_edge(v2, v3, 2.0);

        g1.save_dot("/tmp/test_components.dot");
        auto g2 = Graph<void>::load_dot("/tmp/test_components.dot");

        CHECK(g2.vertex_count() == 4);
        CHECK(g2.edge_count() == 2);

        CHECK(g2.has_edge(v0, v1));
        CHECK(g2.has_edge(v2, v3));
        CHECK_FALSE(g2.has_edge(v0, v2));

        std::remove("/tmp/test_components.dot");
    }

    TEST_CASE("DOT Serialization - Graph with self-loop") {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        g1.add_edge(v0, v0, 1.0); // Self-loop
        g1.add_edge(v0, v1, 2.0);

        g1.save_dot("/tmp/test_selfloop.dot");
        auto g2 = Graph<void>::load_dot("/tmp/test_selfloop.dot");

        CHECK(g2.vertex_count() == 2);
        CHECK(g2.edge_count() == 2);

        CHECK(g2.has_edge(v0, v0)); // Self-loop preserved
        CHECK(g2.has_edge(v0, v1));

        std::remove("/tmp/test_selfloop.dot");
    }

    TEST_CASE("DOT Serialization - Large graph") {
        Graph<void> g1;
        std::vector<Graph<void>::VertexId> vertices;

        // Create 100 vertices
        for (int i = 0; i < 100; i++) {
            vertices.push_back(g1.add_vertex());
        }

        // Create chain of edges
        for (int i = 0; i < 99; i++) {
            g1.add_edge(vertices[i], vertices[i + 1], static_cast<double>(i));
        }

        g1.save_dot("/tmp/test_large.dot");
        auto g2 = Graph<void>::load_dot("/tmp/test_large.dot");

        CHECK(g2.vertex_count() == 100);
        CHECK(g2.edge_count() == 99);

        // Spot check a few edges
        CHECK(g2.has_edge(vertices[0], vertices[1]));
        CHECK(g2.has_edge(vertices[50], vertices[51]));
        CHECK(g2.has_edge(vertices[98], vertices[99]));

        std::remove("/tmp/test_large.dot");
    }

    TEST_CASE("DOT Serialization - Weights preserved") {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        auto v2 = g1.add_vertex();

        g1.add_edge(v0, v1, 0.123456);
        g1.add_edge(v1, v2, 999.999);

        g1.save_dot("/tmp/test_weights.dot");
        auto g2 = Graph<void>::load_dot("/tmp/test_weights.dot");

        auto e1 = g2.get_edge(v0, v1);
        REQUIRE(e1.has_value());
        CHECK(g2.get_weight(e1.value()) == doctest::Approx(0.123456));

        auto e2 = g2.get_edge(v1, v2);
        REQUIRE(e2.has_value());
        CHECK(g2.get_weight(e2.value()) == doctest::Approx(999.999));

        std::remove("/tmp/test_weights.dot");
    }

    TEST_CASE("DOT Serialization - Round trip identity") {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        auto v2 = g1.add_vertex();
        g1.add_edge(v0, v1, 1.5);
        g1.add_edge(v1, v2, 2.5, EdgeType::Directed);

        // First save
        g1.save_dot("/tmp/test_roundtrip1.dot");

        // Load
        auto g2 = Graph<void>::load_dot("/tmp/test_roundtrip1.dot");

        // Second save
        g2.save_dot("/tmp/test_roundtrip2.dot");

        // Load again
        auto g3 = Graph<void>::load_dot("/tmp/test_roundtrip2.dot");

        // Verify all three have same structure
        CHECK(g3.vertex_count() == g1.vertex_count());
        CHECK(g3.edge_count() == g1.edge_count());

        std::remove("/tmp/test_roundtrip1.dot");
        std::remove("/tmp/test_roundtrip2.dot");
    }

    TEST_CASE("DOT Serialization - Graph with properties") {
        Graph<Point2D> g1;
        auto v0 = g1.add_vertex(Point2D(1.0, 2.0));
        auto v1 = g1.add_vertex(Point2D(3.0, 4.0));
        auto v2 = g1.add_vertex(Point2D(5.0, 6.0));

        g1.add_edge(v0, v1, 1.5);
        g1.add_edge(v1, v2, 2.5);

        // Property writer: converts Point2D to string
        auto writer = [](Graph<Point2D>::VertexId id, const Point2D &p) {
            std::ostringstream oss;
            oss << p.x << "," << p.y;
            return oss.str();
        };

        g1.save_dot("/tmp/test_props.dot", writer);

        // Property reader: converts string to Point2D
        auto reader = [](const std::string &s) {
            Point2D p;
            size_t comma_pos = s.find(',');
            if (comma_pos != std::string::npos) {
                p.x = std::stod(s.substr(0, comma_pos));
                p.y = std::stod(s.substr(comma_pos + 1));
            }
            return p;
        };

        auto g2 = Graph<Point2D>::load_dot("/tmp/test_props.dot", reader);

        CHECK(g2.vertex_count() == 3);
        CHECK(g2.edge_count() == 2);

        // Verify properties
        CHECK(g2[v0] == Point2D(1.0, 2.0));
        CHECK(g2[v1] == Point2D(3.0, 4.0));
        CHECK(g2[v2] == Point2D(5.0, 6.0));

        std::remove("/tmp/test_props.dot");
    }

    TEST_CASE("DOT Serialization - Verify file format for undirected") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.5);

        g.save_dot("/tmp/test_format_undirected.dot");

        // Read file and verify format
        std::ifstream in("/tmp/test_format_undirected.dot");
        std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.close();

        // Should use "graph" not "digraph" for pure undirected
        CHECK(content.find("graph G") != std::string::npos);
        CHECK(content.find("digraph") == std::string::npos);

        // Should use "--" for undirected edges
        CHECK(content.find("--") != std::string::npos);

        std::remove("/tmp/test_format_undirected.dot");
    }

    TEST_CASE("DOT Serialization - Verify file format for directed") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        g.add_edge(v0, v1, 1.5, EdgeType::Directed);

        g.save_dot("/tmp/test_format_directed.dot");

        // Read file and verify format
        std::ifstream in("/tmp/test_format_directed.dot");
        std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.close();

        // Should use "digraph" for directed
        CHECK(content.find("digraph G") != std::string::npos);

        // Should use "->" for directed edges
        CHECK(content.find("->") != std::string::npos);

        std::remove("/tmp/test_format_directed.dot");
    }

    TEST_CASE("DOT Serialization - Verify file format for mixed") {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 2.0, EdgeType::Directed);

        g.save_dot("/tmp/test_format_mixed.dot");

        // Read file and verify format
        std::ifstream in("/tmp/test_format_mixed.dot");
        std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        in.close();

        // Should use "digraph" when mixed
        CHECK(content.find("digraph G") != std::string::npos);

        // Should use "->" for all edges in mixed graph
        CHECK(content.find("->") != std::string::npos);

        // Undirected edges should have "dir=none"
        CHECK(content.find("dir=none") != std::string::npos);

        std::remove("/tmp/test_format_mixed.dot");
    }

    TEST_CASE("DOT Serialization - Error handling - invalid file path") {
        Graph<void> g;
        g.add_vertex();

        CHECK_THROWS_AS(g.save_dot("/invalid/path/that/does/not/exist/test.dot"), std::runtime_error);
        CHECK_THROWS_AS(Graph<void>::load_dot("/invalid/path/test.dot"), std::runtime_error);
    }

    TEST_CASE("DOT Serialization - Properties with special characters") {
        Graph<Point2D> g1;
        auto v0 = g1.add_vertex(Point2D(1.5, 2.5));
        auto v1 = g1.add_vertex(Point2D(3.5, 4.5));
        g1.add_edge(v0, v1, 1.0);

        auto writer = [](Graph<Point2D>::VertexId id, const Point2D &p) {
            std::ostringstream oss;
            oss << "Point(" << p.x << "," << p.y << ")";
            return oss.str();
        };

        g1.save_dot("/tmp/test_special.dot", writer);

        // Read file to verify it's valid
        std::ifstream in("/tmp/test_special.dot");
        CHECK(in.is_open());
        in.close();

        auto reader = [](const std::string &s) {
            // Parse "Point(x,y)"
            Point2D p;
            size_t start = s.find('(');
            size_t comma = s.find(',');
            size_t end = s.find(')');
            if (start != std::string::npos && comma != std::string::npos && end != std::string::npos) {
                p.x = std::stod(s.substr(start + 1, comma - start - 1));
                p.y = std::stod(s.substr(comma + 1, end - comma - 1));
            }
            return p;
        };

        auto g2 = Graph<Point2D>::load_dot("/tmp/test_special.dot", reader);

        CHECK(g2.vertex_count() == 2);
        CHECK(g2[v0].x == doctest::Approx(1.5));
        CHECK(g2[v0].y == doctest::Approx(2.5));

        std::remove("/tmp/test_special.dot");
    }

    TEST_CASE("DOT Serialization - Isolated vertices") {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        auto v2 = g1.add_vertex();
        // Only connect v0 and v1, leave v2 isolated
        g1.add_edge(v0, v1, 1.0);

        g1.save_dot("/tmp/test_isolated.dot");
        auto g2 = Graph<void>::load_dot("/tmp/test_isolated.dot");

        CHECK(g2.vertex_count() == 3);
        CHECK(g2.edge_count() == 1);

        CHECK(g2.has_edge(v0, v1));
        CHECK(g2.degree(v2) == 0); // Isolated vertex

        std::remove("/tmp/test_isolated.dot");
    }
}
