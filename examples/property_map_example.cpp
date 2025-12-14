/**
 * Property Map Example
 *
 * Demonstrates the use of Boost.Graph-style property maps for external property storage.
 * Property maps allow multiple properties per vertex/edge without modifying the graph structure.
 *
 * This example shows:
 * - Associative property maps (hash-based, sparse storage)
 * - Vector property maps (array-based, dense storage)
 * - Constant property maps (always return same value)
 * - Identity property maps (return the key itself)
 * - Composite property maps (manage multiple properties)
 * - Graph usage with property maps
 */

#include <graphix/vertex/graph.hpp>
#include <graphix/vertex/property_map.hpp>
#include <iostream>
#include <string>

using namespace graphix::vertex;

void example_basic_property_maps() {
    std::cout << "\n=== Basic Property Maps ===\n";

    // Associative property map - uses hash map internally
    // Good for sparse properties
    auto color_map = make_associative_property_map<size_t, std::string>();

    put(color_map, size_t(0), "red");
    put(color_map, size_t(1), "green");
    put(color_map, size_t(2), "blue");

    std::cout << "Vertex 0 color: " << get(color_map, size_t(0)) << "\n";
    std::cout << "Vertex 1 color: " << get(color_map, size_t(1)) << "\n";
    std::cout << "Vertex 2 color: " << get(color_map, size_t(2)) << "\n";

    // Vector property map - uses vector internally
    // Good for dense properties with contiguous IDs
    auto weight_map = make_vector_property_map<double>(10, 0.0);

    put(weight_map, size_t(0), 1.5);
    put(weight_map, size_t(1), 2.5);
    put(weight_map, size_t(2), 3.5);

    std::cout << "Vertex 0 weight: " << get(weight_map, size_t(0)) << "\n";
    std::cout << "Vertex 1 weight: " << get(weight_map, size_t(1)) << "\n";
    std::cout << "Vertex 2 weight: " << get(weight_map, size_t(2)) << "\n";

    // Constant property map - always returns the same value
    auto default_map = make_constant_property_map<size_t, int>(42);

    std::cout << "Any vertex default value: " << get(default_map, size_t(0)) << "\n";
    std::cout << "Any vertex default value: " << get(default_map, size_t(999)) << "\n";

    // Identity property map - returns the key itself
    auto id_map = make_identity_property_map<size_t>();

    std::cout << "Vertex ID: " << get(id_map, size_t(0)) << "\n";
    std::cout << "Vertex ID: " << get(id_map, size_t(5)) << "\n";
}

void example_composite_property_map() {
    std::cout << "\n=== Composite Property Map ===\n";

    // Create a graph
    Graph<> g;
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    auto e01 = g.add_edge(v0, v1);
    auto e12 = g.add_edge(v1, v2);
    auto e23 = g.add_edge(v2, v3);

    // Composite property map manages multiple properties together
    CompositePropertyMap<> composite;

    // Add vertex properties
    composite.add_vertex_property("color", make_associative_property_map<size_t, std::string>());
    composite.add_vertex_property("visited", make_associative_property_map<size_t, bool>());
    composite.add_vertex_property("distance", make_vector_property_map<int>(10, -1));

    // Add edge properties
    composite.add_edge_property("weight", make_associative_property_map<size_t, double>());
    composite.add_edge_property("label", make_associative_property_map<size_t, std::string>());

    // Set some vertex properties
    auto color_prop = composite.get_vertex_property<std::string>("color");
    put(color_prop, v0, "red");
    put(color_prop, v1, "green");
    put(color_prop, v2, "blue");
    put(color_prop, v3, "yellow");

    auto distance_prop = composite.get_vertex_property<int>("distance");
    put(distance_prop, v0, 0);
    put(distance_prop, v1, 1);
    put(distance_prop, v2, 2);
    put(distance_prop, v3, 3);

    // Set some edge properties
    auto weight_prop = composite.get_edge_property<double>("weight");
    put(weight_prop, e01, 1.5);
    put(weight_prop, e12, 2.0);
    put(weight_prop, e23, 1.0);

    auto label_prop = composite.get_edge_property<std::string>("label");
    put(label_prop, e01, "fast");
    put(label_prop, e12, "medium");
    put(label_prop, e23, "slow");

    // Retrieve and display properties
    std::cout << "Vertex properties:\n";
    for (size_t i = 0; i < 4; ++i) {
        std::cout << "  Vertex " << i << ": color=" << get(color_prop, i) << ", distance=" << get(distance_prop, i)
                  << "\n";
    }

    std::cout << "Edge properties:\n";
    std::cout << "  Edge 0-1: weight=" << get(weight_prop, e01) << ", label=" << get(label_prop, e01) << "\n";
    std::cout << "  Edge 1-2: weight=" << get(weight_prop, e12) << ", label=" << get(label_prop, e12) << "\n";
    std::cout << "  Edge 2-3: weight=" << get(weight_prop, e23) << ", label=" << get(label_prop, e23) << "\n";
}

void example_graph_with_property_maps() {
    std::cout << "\n=== Graph with Property Maps ===\n";

    // Create a simple graph
    //     0
    //    / \
    //   1   2
    //  / \   \
    // 3   4   5
    Graph<> g;
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();
    auto v5 = g.add_vertex();

    g.add_edge(v0, v1);
    g.add_edge(v0, v2);
    g.add_edge(v1, v3);
    g.add_edge(v1, v4);
    g.add_edge(v2, v5);

    // Use property maps to store vertex information
    auto name_map = make_associative_property_map<size_t, std::string>();
    auto level_map = make_vector_property_map<int>(10, -1);
    auto visited_map = make_associative_property_map<size_t, bool>(false);

    // Set vertex names
    put(name_map, v0, "Root");
    put(name_map, v1, "Left");
    put(name_map, v2, "Right");
    put(name_map, v3, "Leaf1");
    put(name_map, v4, "Leaf2");
    put(name_map, v5, "Leaf3");

    // Manually perform a simple BFS-like traversal using property maps
    std::cout << "Manual traversal starting from root:\n";

    // Level 0
    put(level_map, v0, 0);
    put(visited_map, v0, true);
    std::cout << "  Level 0: " << get(name_map, v0) << "\n";

    // Level 1
    put(level_map, v1, 1);
    put(level_map, v2, 1);
    put(visited_map, v1, true);
    put(visited_map, v2, true);
    std::cout << "  Level 1: " << get(name_map, v1) << ", " << get(name_map, v2) << "\n";

    // Level 2
    put(level_map, v3, 2);
    put(level_map, v4, 2);
    put(level_map, v5, 2);
    put(visited_map, v3, true);
    put(visited_map, v4, true);
    put(visited_map, v5, true);
    std::cout << "  Level 2: " << get(name_map, v3) << ", " << get(name_map, v4) << ", " << get(name_map, v5) << "\n";

    std::cout << "\nVertex summary:\n";
    for (size_t vid : {v0, v1, v2, v3, v4, v5}) {
        std::cout << "  " << get(name_map, vid) << " (level=" << get(level_map, vid)
                  << ", visited=" << (get(visited_map, vid) ? "yes" : "no") << ")\n";
    }
}

void example_weighted_graph() {
    std::cout << "\n=== Weighted Graph with Property Maps ===\n";

    // Create a weighted graph using property maps for vertices
    Graph<> g;
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v0, v1);
    g.add_edge(v1, v2);
    g.add_edge(v2, v3);
    g.add_edge(v3, v0);

    // Create vertex value property map
    auto value_map = make_associative_property_map<size_t, double>(0.0);
    put(value_map, v0, 10.5);
    put(value_map, v1, 20.3);
    put(value_map, v2, 15.7);
    put(value_map, v3, 8.9);

    std::cout << "Vertex values:\n";
    for (size_t i = 0; i < 4; ++i) {
        std::cout << "  Vertex " << i << ": value = " << get(value_map, i) << "\n";
    }

    // Calculate total value
    double total_value = 0.0;
    for (size_t i = 0; i < 4; ++i) {
        total_value += get(value_map, i);
    }
    std::cout << "\nTotal vertex value: " << total_value << "\n";
    std::cout << "Average vertex value: " << (total_value / 4.0) << "\n";
}

void example_multiple_properties() {
    std::cout << "\n=== Multiple Properties per Vertex ===\n";

    // Simulate a social network with multiple vertex properties
    Graph<> g;
    auto alice = g.add_vertex();
    auto bob = g.add_vertex();
    auto carol = g.add_vertex();
    auto dave = g.add_vertex();

    g.add_edge(alice, bob);
    g.add_edge(alice, carol);
    g.add_edge(bob, carol);
    g.add_edge(carol, dave);

    // Create multiple property maps for different attributes
    auto name_map = make_associative_property_map<size_t, std::string>();
    auto age_map = make_associative_property_map<size_t, int>();
    auto active_map = make_associative_property_map<size_t, bool>();
    auto score_map = make_associative_property_map<size_t, double>();

    // Set properties for each person
    put(name_map, alice, "Alice");
    put(age_map, alice, 25);
    put(active_map, alice, true);
    put(score_map, alice, 4.5);

    put(name_map, bob, "Bob");
    put(age_map, bob, 30);
    put(active_map, bob, true);
    put(score_map, bob, 4.2);

    put(name_map, carol, "Carol");
    put(age_map, carol, 28);
    put(active_map, carol, false);
    put(score_map, carol, 3.8);

    put(name_map, dave, "Dave");
    put(age_map, dave, 35);
    put(active_map, dave, true);
    put(score_map, dave, 4.7);

    // Display all properties
    std::cout << "User profiles:\n";
    for (size_t user : {alice, bob, carol, dave}) {
        std::cout << "  " << get(name_map, user) << " (age: " << get(age_map, user)
                  << ", active: " << (get(active_map, user) ? "yes" : "no") << ", score: " << get(score_map, user)
                  << ")\n";
    }

    // Compute average age of active users
    double total_age = 0;
    int active_count = 0;
    for (size_t user : {alice, bob, carol, dave}) {
        if (get(active_map, user)) {
            total_age += get(age_map, user);
            active_count++;
        }
    }

    std::cout << "\nAverage age of active users: " << (total_age / active_count) << "\n";
    std::cout << "Number of active users: " << active_count << " out of 4\n";
}

int main() {
    std::cout << "=================================================\n";
    std::cout << "   Property Map Examples for Graphix\n";
    std::cout << "=================================================\n";

    example_basic_property_maps();
    example_composite_property_map();
    example_graph_with_property_maps();
    example_weighted_graph();
    example_multiple_properties();

    std::cout << "\n=================================================\n";
    std::cout << "All examples completed successfully!\n";
    std::cout << "=================================================\n";

    return 0;
}
