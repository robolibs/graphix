#include "graphix/vertex/algorithms/cycle_detection.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>
#include <string>

using namespace graphix::vertex;

void print_cycle(const CycleResult &result, const Graph<std::string> &g) {
    if (result.has_cycle) {
        std::cout << "  Cycle found: ";
        for (size_t i = 0; i < result.cycle.size(); i++) {
            std::cout << g[result.cycle[i]];
            if (i < result.cycle.size() - 1) {
                std::cout << " -> ";
            }
        }
        std::cout << "\n";
    } else {
        std::cout << "  No cycle detected.\n";
    }
}

void print_cycle_void(const CycleResult &result) {
    if (result.has_cycle) {
        std::cout << "  Cycle found: ";
        for (size_t i = 0; i < result.cycle.size(); i++) {
            std::cout << "v" << result.cycle[i];
            if (i < result.cycle.size() - 1) {
                std::cout << " -> ";
            }
        }
        std::cout << "\n";
    } else {
        std::cout << "  No cycle detected.\n";
    }
}

int main() {
    std::cout << "=== Cycle Detection Examples ===\n\n";

    // Example 1: Module Dependencies (Directed Graph)
    std::cout << "Example 1: Detecting Circular Dependencies in Modules\n";
    std::cout << "-------------------------------------------------------\n";
    {
        Graph<std::string> modules;

        auto mod_a = modules.add_vertex("Module A");
        auto mod_b = modules.add_vertex("Module B");
        auto mod_c = modules.add_vertex("Module C");
        auto mod_d = modules.add_vertex("Module D");

        // A depends on B, B depends on C, C depends on A (circular!)
        modules.add_edge(mod_a, mod_b, 1.0, EdgeType::Directed);
        modules.add_edge(mod_b, mod_c, 1.0, EdgeType::Directed);
        modules.add_edge(mod_c, mod_a, 1.0, EdgeType::Directed);
        modules.add_edge(mod_b, mod_d, 1.0, EdgeType::Directed); // D has no cycle

        std::cout << "Dependencies: A->B, B->C, C->A, B->D\n";
        auto result = find_cycle_directed(modules);
        print_cycle(result, modules);
        std::cout << "This circular dependency must be broken!\n";
    }

    std::cout << "\n";

    // Example 2: Valid Build System (No Cycles)
    std::cout << "Example 2: Valid Build System (DAG)\n";
    std::cout << "------------------------------------\n";
    {
        Graph<std::string> build;

        auto src = build.add_vertex("source.cpp");
        auto hdr = build.add_vertex("header.h");
        auto obj = build.add_vertex("object.o");
        auto exe = build.add_vertex("executable");

        build.add_edge(src, obj, 1.0, EdgeType::Directed);
        build.add_edge(hdr, obj, 1.0, EdgeType::Directed);
        build.add_edge(obj, exe, 1.0, EdgeType::Directed);

        std::cout << "Dependencies: source->object, header->object, object->executable\n";
        auto result = find_cycle_directed(build);
        print_cycle(result, build);
        std::cout << "Build system is valid - no circular dependencies!\n";
    }

    std::cout << "\n";

    // Example 3: Network Topology (Undirected Graph)
    std::cout << "Example 3: Network Loop Detection\n";
    std::cout << "----------------------------------\n";
    {
        Graph<std::string> network;

        auto router_a = network.add_vertex("Router A");
        auto router_b = network.add_vertex("Router B");
        auto router_c = network.add_vertex("Router C");
        auto router_d = network.add_vertex("Router D");

        // Create a network with a loop: A-B-C-A
        network.add_edge(router_a, router_b, 1.0, EdgeType::Undirected);
        network.add_edge(router_b, router_c, 1.0, EdgeType::Undirected);
        network.add_edge(router_c, router_a, 1.0, EdgeType::Undirected);
        network.add_edge(router_c, router_d, 1.0, EdgeType::Undirected);

        std::cout << "Network: A-B, B-C, C-A, C-D\n";
        auto result = find_cycle_undirected(network);
        print_cycle(result, network);
        std::cout << "Network has redundant paths (loop exists).\n";
        std::cout << "This can provide fault tolerance!\n";
    }

    std::cout << "\n";

    // Example 4: Tree Structure (No Cycles)
    std::cout << "Example 4: Tree Structure Validation\n";
    std::cout << "-------------------------------------\n";
    {
        Graph<std::string> tree;

        auto root = tree.add_vertex("Root");
        auto left = tree.add_vertex("Left Child");
        auto right = tree.add_vertex("Right Child");
        auto left_left = tree.add_vertex("Left-Left");
        auto left_right = tree.add_vertex("Left-Right");

        tree.add_edge(root, left, 1.0, EdgeType::Undirected);
        tree.add_edge(root, right, 1.0, EdgeType::Undirected);
        tree.add_edge(left, left_left, 1.0, EdgeType::Undirected);
        tree.add_edge(left, left_right, 1.0, EdgeType::Undirected);

        std::cout << "Structure: Root connected to children in tree pattern\n";
        auto result = find_cycle_undirected(tree);
        print_cycle(result, tree);
        std::cout << "Valid tree - no cycles!\n";
    }

    std::cout << "\n";

    // Example 5: Deadlock Detection
    std::cout << "Example 5: Deadlock Detection in Resource Allocation\n";
    std::cout << "-----------------------------------------------------\n";
    {
        Graph<std::string> resources;

        auto proc1 = resources.add_vertex("Process 1");
        auto proc2 = resources.add_vertex("Process 2");
        auto proc3 = resources.add_vertex("Process 3");

        // Process 1 waits for Process 2
        // Process 2 waits for Process 3
        // Process 3 waits for Process 1 (deadlock!)
        resources.add_edge(proc1, proc2, 1.0, EdgeType::Directed);
        resources.add_edge(proc2, proc3, 1.0, EdgeType::Directed);
        resources.add_edge(proc3, proc1, 1.0, EdgeType::Directed);

        std::cout << "Wait-for graph: P1->P2, P2->P3, P3->P1\n";
        auto result = find_cycle_directed(resources);
        print_cycle(result, resources);

        if (result.has_cycle) {
            std::cout << "DEADLOCK DETECTED! System is stuck.\n";
            std::cout << "Need to kill one process to break the cycle.\n";
        }
    }

    std::cout << "\n";

    // Example 6: Course Prerequisites Validation
    std::cout << "Example 6: Valid Course Prerequisites\n";
    std::cout << "--------------------------------------\n";
    {
        Graph<std::string> courses;

        auto intro = courses.add_vertex("Intro CS");
        auto data_struct = courses.add_vertex("Data Structures");
        auto algorithms = courses.add_vertex("Algorithms");
        auto ai = courses.add_vertex("AI");

        courses.add_edge(intro, data_struct, 1.0, EdgeType::Directed);
        courses.add_edge(data_struct, algorithms, 1.0, EdgeType::Directed);
        courses.add_edge(algorithms, ai, 1.0, EdgeType::Directed);

        std::cout << "Prerequisites: Intro->DataStruct->Algorithms->AI\n";
        auto result = find_cycle_directed(courses);
        print_cycle(result, courses);
        std::cout << "Course sequence is valid!\n";
    }

    std::cout << "\n";

    // Example 7: Social Network Clique Detection
    std::cout << "Example 7: Friend Circle with Triangle\n";
    std::cout << "---------------------------------------\n";
    {
        Graph<std::string> friends;

        auto alice = friends.add_vertex("Alice");
        auto bob = friends.add_vertex("Bob");
        auto carol = friends.add_vertex("Carol");
        auto dave = friends.add_vertex("Dave");

        // Alice-Bob-Carol-Alice forms a triangle (cycle)
        friends.add_edge(alice, bob, 1.0, EdgeType::Undirected);
        friends.add_edge(bob, carol, 1.0, EdgeType::Undirected);
        friends.add_edge(carol, alice, 1.0, EdgeType::Undirected);
        friends.add_edge(carol, dave, 1.0, EdgeType::Undirected);

        std::cout << "Friendships: Alice-Bob, Bob-Carol, Carol-Alice, Carol-Dave\n";
        auto result = find_cycle_undirected(friends);
        print_cycle(result, friends);
        std::cout << "Found a tight-knit friend group (triangle)!\n";
    }

    std::cout << "\n";

    // Example 8: Using the general find_cycle (auto-detect)
    std::cout << "Example 8: Auto-detect Graph Type\n";
    std::cout << "----------------------------------\n";
    {
        Graph<void> g1;
        auto v0 = g1.add_vertex();
        auto v1 = g1.add_vertex();
        auto v2 = g1.add_vertex();

        g1.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g1.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g1.add_edge(v2, v0, 1.0, EdgeType::Directed);

        std::cout << "Directed graph with cycle:\n";
        auto result1 = find_cycle(g1); // Auto-detects directed
        print_cycle_void(result1);

        Graph<void> g2;
        auto u0 = g2.add_vertex();
        auto u1 = g2.add_vertex();
        auto u2 = g2.add_vertex();

        g2.add_edge(u0, u1, 1.0, EdgeType::Undirected);
        g2.add_edge(u1, u2, 1.0, EdgeType::Undirected);

        std::cout << "\nUndirected graph (tree):\n";
        auto result2 = find_cycle(g2); // Auto-detects undirected
        print_cycle_void(result2);
    }

    std::cout << "\n";

    // Example 9: Performance - Large Graph
    std::cout << "Example 9: Large Graph Cycle Detection\n";
    std::cout << "---------------------------------------\n";
    {
        Graph<void> large;
        std::vector<size_t> vertices;

        // Create 100 vertices
        for (int i = 0; i < 100; i++) {
            vertices.push_back(large.add_vertex());
        }

        // Create a long chain
        for (int i = 0; i < 99; i++) {
            large.add_edge(vertices[i], vertices[i + 1], 1.0, EdgeType::Directed);
        }

        // Add back edge to create cycle
        large.add_edge(vertices[99], vertices[50], 1.0, EdgeType::Directed);

        std::cout << "Chain of 100 vertices with back edge from v99 to v50\n";
        auto result = find_cycle_directed(large);

        if (result.has_cycle) {
            std::cout << "  Cycle detected! Cycle length: " << result.cycle.size() << " vertices\n";
        }
    }

    std::cout << "\n";

    // Example 10: Practical Use - Validating Import Statements
    std::cout << "Example 10: Python Import Validation\n";
    std::cout << "-------------------------------------\n";
    {
        Graph<std::string> imports;

        auto main = imports.add_vertex("main.py");
        auto utils = imports.add_vertex("utils.py");
        auto models = imports.add_vertex("models.py");
        auto config = imports.add_vertex("config.py");

        // main imports utils
        // utils imports models
        // models imports config
        // config imports utils (circular import!)
        imports.add_edge(main, utils, 1.0, EdgeType::Directed);
        imports.add_edge(utils, models, 1.0, EdgeType::Directed);
        imports.add_edge(models, config, 1.0, EdgeType::Directed);
        imports.add_edge(config, utils, 1.0, EdgeType::Directed);

        std::cout << "Import chain: main->utils->models->config->utils\n";
        auto result = find_cycle_directed(imports);

        if (result.has_cycle) {
            std::cout << "  ❌ CIRCULAR IMPORT DETECTED:\n";
            print_cycle(result, imports);
            std::cout << "  This will cause an ImportError at runtime!\n";
            std::cout << "  Refactor to break the circular dependency.\n";
        }
    }

    return 0;
}
