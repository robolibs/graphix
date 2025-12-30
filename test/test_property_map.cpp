#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include "graphix/vertex/graph.hpp"
#include "graphix/vertex/property_map.hpp"
#include <string>

using namespace graphix::vertex;

TEST_SUITE("Property Map - Associative") {
    TEST_CASE("Basic get/put operations") {
        AssociativePropertyMap<size_t, std::string> pmap;

        pmap.put(0, "vertex0");
        pmap.put(1, "vertex1");
        pmap.put(2, "vertex2");

        CHECK(pmap.get(0) == "vertex0");
        CHECK(pmap.get(1) == "vertex1");
        CHECK(pmap.get(2) == "vertex2");
        CHECK(pmap.size() == 3);
    }

    TEST_CASE("Free function interface") {
        auto pmap = make_associative_property_map<size_t, int>();

        put(pmap, 0, 100);
        put(pmap, 1, 200);
        put(pmap, 2, 300);

        CHECK(get(pmap, 0) == 100);
        CHECK(get(pmap, 1) == 200);
        CHECK(get(pmap, 2) == 300);
    }

    TEST_CASE("Contains check") {
        AssociativePropertyMap<size_t, double> pmap;

        pmap.put(0, 1.5);
        pmap.put(5, 2.5);

        CHECK(pmap.contains(0));
        CHECK(pmap.contains(5));
        CHECK_FALSE(pmap.contains(1));
        CHECK_FALSE(pmap.contains(10));
    }

    TEST_CASE("Default value") {
        AssociativePropertyMap<size_t, int> pmap(999);

        pmap.put(0, 100);

        CHECK(pmap.get(0) == 100);
        CHECK(pmap.get(1) == 999);   // Default
        CHECK(pmap.get(100) == 999); // Default
    }

    TEST_CASE("Erase operation") {
        AssociativePropertyMap<size_t, std::string> pmap;

        pmap.put(0, "A");
        pmap.put(1, "B");
        pmap.put(2, "C");

        CHECK(pmap.size() == 3);

        pmap.erase(1);

        CHECK(pmap.size() == 2);
        CHECK(pmap.contains(0));
        CHECK_FALSE(pmap.contains(1));
        CHECK(pmap.contains(2));
    }

    TEST_CASE("Clear operation") {
        AssociativePropertyMap<size_t, int> pmap;

        pmap.put(0, 1);
        pmap.put(1, 2);
        pmap.put(2, 3);

        CHECK(pmap.size() == 3);

        pmap.clear();

        CHECK(pmap.size() == 0);
        CHECK_FALSE(pmap.contains(0));
    }

    TEST_CASE("Iterator support") {
        AssociativePropertyMap<size_t, int> pmap;

        pmap.put(0, 10);
        pmap.put(1, 20);
        pmap.put(2, 30);

        int sum = 0;
        for (const auto &[key, value] : pmap) {
            sum += value;
        }

        CHECK(sum == 60);
    }

    TEST_CASE("Update existing value") {
        AssociativePropertyMap<size_t, std::string> pmap;

        pmap.put(0, "initial");
        CHECK(pmap.get(0) == "initial");

        pmap.put(0, "updated");
        CHECK(pmap.get(0) == "updated");
        CHECK(pmap.size() == 1); // Still one entry
    }
}

TEST_SUITE("Property Map - Vector") {
    TEST_CASE("Basic get/put operations") {
        VectorPropertyMap<int> pmap(10, 0);

        pmap.put(0, 100);
        pmap.put(5, 500);
        pmap.put(9, 900);

        CHECK(pmap.get(0) == 100);
        CHECK(pmap.get(5) == 500);
        CHECK(pmap.get(9) == 900);
    }

    TEST_CASE("Auto-resize on put") {
        VectorPropertyMap<double> pmap(5, 0.0);

        CHECK(pmap.size() == 5);

        pmap.put(10, 3.14);

        CHECK(pmap.size() == 11);
        CHECK(pmap.get(10) == 3.14);
        CHECK(pmap.get(7) == 0.0); // Default value
    }

    TEST_CASE("Default values") {
        VectorPropertyMap<int> pmap(5, 999);

        // All values should be default
        for (size_t i = 0; i < 5; i++) {
            CHECK(pmap.get(i) == 999);
        }

        pmap.put(2, 42);
        CHECK(pmap.get(2) == 42);
        CHECK(pmap.get(3) == 999);
    }

    TEST_CASE("Reserve capacity") {
        VectorPropertyMap<int> pmap;

        pmap.reserve(100);

        for (size_t i = 0; i < 50; i++) {
            pmap.put(i, static_cast<int>(i * 10));
        }

        CHECK(pmap.get(25) == 250);
    }

    TEST_CASE("Resize operation") {
        VectorPropertyMap<int> pmap(5, 0);

        pmap.put(2, 42);

        pmap.resize(10);

        CHECK(pmap.size() == 10);
        CHECK(pmap.get(2) == 42); // Preserved
        CHECK(pmap.get(7) == 0);  // New default
    }

    TEST_CASE("Clear operation") {
        VectorPropertyMap<int> pmap(10, 0);

        for (size_t i = 0; i < 10; i++) {
            pmap.put(i, static_cast<int>(i));
        }

        CHECK(pmap.size() == 10);

        pmap.clear();

        CHECK(pmap.size() == 0);
    }

    TEST_CASE("Contains check") {
        VectorPropertyMap<int> pmap(5, 0);

        CHECK(pmap.contains(0));
        CHECK(pmap.contains(4));
        CHECK_FALSE(pmap.contains(10));
    }
}

TEST_SUITE("Property Map - Constant") {
    TEST_CASE("Always returns same value") {
        ConstantPropertyMap<size_t, int> pmap(42);

        CHECK(pmap.get(0) == 42);
        CHECK(pmap.get(100) == 42);
        CHECK(pmap.get(999) == 42);
    }

    TEST_CASE("Put operations ignored") {
        ConstantPropertyMap<size_t, std::string> pmap("constant");

        pmap.put(0, "try to change");

        CHECK(pmap.get(0) == "constant");
        CHECK(pmap.get(1) == "constant");
    }

    TEST_CASE("Always contains any key") {
        ConstantPropertyMap<size_t, double> pmap(3.14);

        CHECK(pmap.contains(0));
        CHECK(pmap.contains(999));
        CHECK(pmap.contains(123456));
    }

    TEST_CASE("Factory function") {
        auto pmap = make_constant_property_map<size_t, int>(100);

        CHECK(get(pmap, 0) == 100);
        CHECK(get(pmap, 999) == 100);
    }
}

TEST_SUITE("Property Map - Identity") {
    TEST_CASE("Returns key as value") {
        IdentityPropertyMap<size_t> pmap;

        CHECK(pmap.get(0) == 0);
        CHECK(pmap.get(42) == 42);
        CHECK(pmap.get(999) == 999);
    }

    TEST_CASE("Put operations ignored") {
        IdentityPropertyMap<int> pmap;

        pmap.put(10, 999);

        CHECK(pmap.get(10) == 10); // Still returns key
    }

    TEST_CASE("Always contains") {
        IdentityPropertyMap<size_t> pmap;

        CHECK(pmap.contains(0));
        CHECK(pmap.contains(12345));
    }

    TEST_CASE("Factory function") {
        auto pmap = make_identity_property_map<size_t>();

        CHECK(get(pmap, 5) == 5);
        CHECK(get(pmap, 100) == 100);
    }
}

TEST_SUITE("Property Map - Composite") {
    TEST_CASE("Add and retrieve vertex properties") {
        CompositePropertyMap<> composite;

        auto colors = make_associative_property_map<size_t, std::string>();
        auto weights = make_associative_property_map<size_t, double>();

        composite.add_vertex_property("color", colors);
        composite.add_vertex_property("weight", weights);

        CHECK(composite.has_vertex_property("color"));
        CHECK(composite.has_vertex_property("weight"));
        CHECK_FALSE(composite.has_vertex_property("missing"));
    }

    TEST_CASE("Add and retrieve edge properties") {
        CompositePropertyMap<> composite;

        auto costs = make_associative_property_map<size_t, int>();
        auto labels = make_associative_property_map<size_t, std::string>();

        composite.add_edge_property("cost", costs);
        composite.add_edge_property("label", labels);

        CHECK(composite.has_edge_property("cost"));
        CHECK(composite.has_edge_property("label"));
        CHECK_FALSE(composite.has_edge_property("missing"));
    }

    TEST_CASE("Use retrieved property maps") {
        CompositePropertyMap<> composite;

        auto colors = make_associative_property_map<size_t, std::string>();
        composite.add_vertex_property("color", colors);

        auto retrieved = composite.get_vertex_property<std::string>("color");
        REQUIRE(retrieved != nullptr);

        put(retrieved, 0, "red");
        put(retrieved, 1, "blue");

        CHECK(get(retrieved, 0) == "red");
        CHECK(get(retrieved, 1) == "blue");
    }

    TEST_CASE("Remove properties") {
        CompositePropertyMap<> composite;

        auto colors = make_associative_property_map<size_t, std::string>();
        composite.add_vertex_property("color", colors);

        CHECK(composite.has_vertex_property("color"));

        composite.remove_vertex_property("color");

        CHECK_FALSE(composite.has_vertex_property("color"));
    }

    TEST_CASE("Clear all properties") {
        CompositePropertyMap<> composite;

        auto colors = make_associative_property_map<size_t, std::string>();
        auto weights = make_associative_property_map<size_t, double>();
        auto costs = make_associative_property_map<size_t, int>();

        composite.add_vertex_property("color", colors);
        composite.add_vertex_property("weight", weights);
        composite.add_edge_property("cost", costs);

        CHECK(composite.has_vertex_property("color"));
        CHECK(composite.has_edge_property("cost"));

        composite.clear();

        CHECK_FALSE(composite.has_vertex_property("color"));
        CHECK_FALSE(composite.has_vertex_property("weight"));
        CHECK_FALSE(composite.has_edge_property("cost"));
    }

    TEST_CASE("Multiple properties on same graph") {
        CompositePropertyMap<> composite;

        auto names = make_associative_property_map<size_t, std::string>();
        auto distances = make_vector_property_map<double>(10, 0.0);
        auto visited = make_associative_property_map<size_t, bool>(false);

        composite.add_vertex_property("name", names);
        composite.add_vertex_property("distance", distances);
        composite.add_vertex_property("visited", visited);

        // Use names
        auto name_map = composite.get_vertex_property<std::string>("name");
        put(name_map, 0, "Start");
        put(name_map, 5, "Goal");

        // Use distances
        auto dist_map = composite.get_vertex_property<double>("distance");
        put(dist_map, 0, 0.0);
        put(dist_map, 5, 100.5);

        // Use visited
        auto vis_map = composite.get_vertex_property<bool>("visited");
        put(vis_map, 0, true);

        CHECK(get(name_map, 0) == "Start");
        CHECK(get(dist_map, 5) == 100.5);
        CHECK(get(vis_map, 0) == true);
        CHECK(get(vis_map, 5) == false); // Default
    }
}

TEST_SUITE("Property Map - Integration with Graph") {
    TEST_CASE("Vertex colors in BFS-style algorithm") {
        Graph<void> g;

        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Undirected);
        g.add_edge(v1, v2, 1.0, EdgeType::Undirected);
        g.add_edge(v2, v3, 1.0, EdgeType::Undirected);

        // Create color property map
        auto colors = make_associative_property_map<size_t, std::string>("white");

        // Simulate BFS coloring
        put(colors, v0, "gray"); // Discovered
        put(colors, v1, "gray");
        put(colors, v2, "black"); // Finished

        CHECK(get(colors, v0) == "gray");
        CHECK(get(colors, v2) == "black");
        CHECK(get(colors, v3) == "white"); // Default
    }

    TEST_CASE("Edge weights as external property") {
        Graph<void> g;

        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        auto e0 = g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        auto e1 = g.add_edge(v1, v2, 1.0, EdgeType::Directed);

        // External edge weights (in addition to built-in)
        auto capacities = make_associative_property_map<size_t, int>();

        put(capacities, e0, 100);
        put(capacities, e1, 50);

        CHECK(get(capacities, e0) == 100);
        CHECK(get(capacities, e1) == 50);
    }

    TEST_CASE("Multiple vertex properties") {
        Graph<std::string> g;

        auto alice = g.add_vertex("Alice");
        auto bob = g.add_vertex("Bob");
        auto carol = g.add_vertex("Carol");

        // External properties
        auto ages = make_associative_property_map<size_t, int>();
        auto cities = make_associative_property_map<size_t, std::string>();

        put(ages, alice, 30);
        put(ages, bob, 25);
        put(ages, carol, 35);

        put(cities, alice, "NYC");
        put(cities, bob, "LA");
        put(cities, carol, "Chicago");

        CHECK(g[alice] == "Alice");      // Built-in property
        CHECK(get(ages, alice) == 30);   // External property
        CHECK(get(cities, bob) == "LA"); // Another external property
    }
}
