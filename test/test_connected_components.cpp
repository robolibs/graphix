#include "graphix/vertex/algorithms/connected_components.hpp"
#include "graphix/vertex/graph.hpp"
#include <doctest/doctest.h>

using namespace graphix::vertex;

TEST_CASE("Connected Components - Single component") {
    Graph<void> g;

    // Create connected graph: 0 - 1 - 2 - 3
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v0, v1);
    g.add_edge(v1, v2);
    g.add_edge(v2, v3);

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 1);
    CHECK(result.components.size() == 1);
    CHECK(result.components[0].size() == 4);

    // All vertices should have same component ID
    CHECK(result.component_id[v0] == result.component_id[v1]);
    CHECK(result.component_id[v1] == result.component_id[v2]);
    CHECK(result.component_id[v2] == result.component_id[v3]);

    CHECK(algorithms::is_connected(g) == true);
}

TEST_CASE("Connected Components - Two components") {
    Graph<void> g;

    // Component 1: 0 - 1
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    g.add_edge(v0, v1);

    // Component 2: 2 - 3
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    g.add_edge(v2, v3);

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 2);
    CHECK(result.components.size() == 2);

    // Vertices in same component should have same ID
    CHECK(result.component_id[v0] == result.component_id[v1]);
    CHECK(result.component_id[v2] == result.component_id[v3]);

    // Vertices in different components should have different IDs
    CHECK(result.component_id[v0] != result.component_id[v2]);

    CHECK(algorithms::is_connected(g) == false);

    // Check same_component helper
    CHECK(algorithms::same_component(g, v0, v1) == true);
    CHECK(algorithms::same_component(g, v2, v3) == true);
    CHECK(algorithms::same_component(g, v0, v2) == false);
}

TEST_CASE("Connected Components - Multiple components") {
    Graph<void> g;

    // Component 1: 0 - 1 - 2
    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    g.add_edge(v0, v1);
    g.add_edge(v1, v2);

    // Component 2: 3 - 4
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();
    g.add_edge(v3, v4);

    // Component 3: 5 (isolated)
    auto v5 = g.add_vertex();

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 3);
    CHECK(result.components.size() == 3);

    // Check component sizes
    bool found_size_3 = false;
    bool found_size_2 = false;
    bool found_size_1 = false;

    for (const auto &component : result.components) {
        if (component.size() == 3)
            found_size_3 = true;
        if (component.size() == 2)
            found_size_2 = true;
        if (component.size() == 1)
            found_size_1 = true;
    }

    CHECK(found_size_3);
    CHECK(found_size_2);
    CHECK(found_size_1);

    CHECK(algorithms::largest_component_size(g) == 3);
}

TEST_CASE("Connected Components - Cycle") {
    Graph<void> g;

    // Square: 0 - 1
    //         |   |
    //         3 - 2

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();

    g.add_edge(v0, v1);
    g.add_edge(v1, v2);
    g.add_edge(v2, v3);
    g.add_edge(v3, v0);

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 1);
    CHECK(result.components[0].size() == 4);
    CHECK(algorithms::is_connected(g) == true);
}

TEST_CASE("Connected Components - Empty graph") {
    Graph<void> g;

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 0);
    CHECK(result.components.empty());
    CHECK(algorithms::is_connected(g) == true); // Empty graph considered connected
}

TEST_CASE("Connected Components - Single vertex") {
    Graph<void> g;
    auto v0 = g.add_vertex();

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 1);
    CHECK(result.components.size() == 1);
    CHECK(result.components[0].size() == 1);
    CHECK(algorithms::is_connected(g) == true);
}

TEST_CASE("Connected Components - All isolated vertices") {
    Graph<void> g;

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 3);
    CHECK(result.components.size() == 3);

    // Each component should have size 1
    for (const auto &component : result.components) {
        CHECK(component.size() == 1);
    }

    CHECK(algorithms::is_connected(g) == false);
    CHECK(algorithms::largest_component_size(g) == 1);
}

TEST_CASE("Connected Components - Complex graph") {
    Graph<void> g;

    //     0---1       5---6
    //     |   |       |
    //     2---3       7
    //
    //         4

    auto v0 = g.add_vertex();
    auto v1 = g.add_vertex();
    auto v2 = g.add_vertex();
    auto v3 = g.add_vertex();
    auto v4 = g.add_vertex();
    auto v5 = g.add_vertex();
    auto v6 = g.add_vertex();
    auto v7 = g.add_vertex();

    // Component 1: square
    g.add_edge(v0, v1);
    g.add_edge(v1, v3);
    g.add_edge(v3, v2);
    g.add_edge(v2, v0);

    // Component 2: triangle
    g.add_edge(v5, v6);
    g.add_edge(v5, v7);

    // Component 3: isolated v4

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 3);

    // Find the component sizes
    std::vector<size_t> sizes;
    for (const auto &component : result.components) {
        sizes.push_back(component.size());
    }
    std::sort(sizes.begin(), sizes.end());

    CHECK(sizes[0] == 1); // v4
    CHECK(sizes[1] == 3); // v5, v6, v7
    CHECK(sizes[2] == 4); // v0, v1, v2, v3

    CHECK(algorithms::largest_component_size(g) == 4);
}

TEST_CASE("Connected Components - Graph with properties") {
    struct Node {
        std::string name;
    };

    Graph<Node> g;

    auto v0 = g.add_vertex({"A"});
    auto v1 = g.add_vertex({"B"});
    auto v2 = g.add_vertex({"C"});
    auto v3 = g.add_vertex({"D"});

    g.add_edge(v0, v1);
    g.add_edge(v2, v3);

    auto result = algorithms::connected_components(g);

    CHECK(result.num_components == 2);

    // Verify properties are still accessible
    CHECK(g[v0].name == "A");
    CHECK(g[v1].name == "B");
}
