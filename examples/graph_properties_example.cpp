#include "graphix/vertex/algorithms/graph_properties.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>
#include <string>
#include <vector>

using namespace graphix::vertex;
using namespace graphix::vertex::algorithms;

// Example 1: Bipartite Graph - Job Assignment
void example_bipartite_job_assignment() {
    std::cout << "\n=== Example 1: Bipartite Graph - Job Assignment ===\n";
    std::cout << "Scenario: Matching workers to tasks\n";
    std::cout << "Workers: Alice(0), Bob(1), Carol(2)\n";
    std::cout << "Tasks: Documentation(3), Testing(4), Development(5)\n\n";

    Graph<void> job_graph;

    // Add vertices (workers 0-2, tasks 3-5)
    auto alice = job_graph.add_vertex();
    auto bob = job_graph.add_vertex();
    auto carol = job_graph.add_vertex();
    auto docs = job_graph.add_vertex();
    auto testing = job_graph.add_vertex();
    auto dev = job_graph.add_vertex();

    // Connect workers to tasks they can do
    job_graph.add_edge(alice, docs);    // Alice can do docs
    job_graph.add_edge(alice, testing); // Alice can do testing
    job_graph.add_edge(bob, dev);       // Bob can do development
    job_graph.add_edge(carol, testing); // Carol can do testing
    job_graph.add_edge(carol, dev);     // Carol can do development

    auto result = is_bipartite(job_graph);
    if (result.is_bipartite) {
        std::cout << "✓ Graph is bipartite - valid job assignment possible!\n";
        std::cout << "  Workers partition: ";
        for (size_t i = 0; i <= 2; ++i) {
            if (result.coloring[i] == 0) {
                std::cout << i << " ";
            }
        }
        std::cout << "\n  Tasks partition: ";
        for (size_t i = 3; i <= 5; ++i) {
            if (result.coloring.count(i) && result.coloring[i] == 1) {
                std::cout << i << " ";
            }
        }
        std::cout << "\n";
    } else {
        std::cout << "✗ Graph is not bipartite - conflicting assignments!\n";
    }
}

// Example 2: Bipartite Graph - Conflict Detection
void example_bipartite_conflict() {
    std::cout << "\n=== Example 2: Bipartite Graph - Detecting Conflicts ===\n";
    std::cout << "Scenario: Course scheduling (students ↔ time slots)\n\n";

    Graph<void> schedule;

    // Students
    auto s1 = schedule.add_vertex();
    auto s2 = schedule.add_vertex();
    auto s3 = schedule.add_vertex();

    // Time slots
    auto t1 = schedule.add_vertex();
    auto t2 = schedule.add_vertex();

    // Student preferences
    schedule.add_edge(s1, t1);
    schedule.add_edge(s2, t2);
    schedule.add_edge(s3, t1);

    // CONFLICT: Adding edge between students creates odd cycle
    schedule.add_edge(s1, s2);

    auto result = is_bipartite(schedule);
    if (result.is_bipartite) {
        std::cout << "✓ Schedule is valid (bipartite)\n";
    } else {
        std::cout << "✗ Schedule has conflicts! (not bipartite)\n";
        if (!result.odd_cycle.empty()) {
            std::cout << "  Odd cycle detected: ";
            for (auto v : result.odd_cycle) {
                std::cout << v << " ";
            }
            std::cout << "\n";
        }
    }
}

// Example 3: Acyclic Check - DAG Validation for Build System
void example_dag_build_system() {
    std::cout << "\n=== Example 3: DAG Validation - Build System Dependencies ===\n";
    std::cout << "Scenario: Checking for circular dependencies in a build graph\n";
    std::cout << "Modules: main → utils → logger\n\n";

    Graph<void> build_graph;

    auto main_mod = build_graph.add_vertex();
    auto utils_mod = build_graph.add_vertex();
    auto logger_mod = build_graph.add_vertex();

    // Valid dependencies (directed edges)
    build_graph.add_edge(main_mod, utils_mod, 1.0, EdgeType::Directed);
    build_graph.add_edge(utils_mod, logger_mod, 1.0, EdgeType::Directed);

    if (is_acyclic(build_graph)) {
        std::cout << "✓ Build graph is a DAG - no circular dependencies!\n";
        std::cout << "  Build order is well-defined.\n";
    } else {
        std::cout << "✗ Build graph has cycles - circular dependency detected!\n";
    }

    // Now create a circular dependency
    std::cout << "\nAdding circular dependency: logger → main\n";
    build_graph.add_edge(logger_mod, main_mod, 1.0, EdgeType::Directed);

    if (is_acyclic(build_graph)) {
        std::cout << "✓ Build graph is still a DAG\n";
    } else {
        std::cout << "✗ Build graph now has cycles!\n";
        std::cout << "  Cannot determine valid build order.\n";
    }
}

// Example 4: Acyclic Check - Tree Detection
void example_tree_detection() {
    std::cout << "\n=== Example 4: Tree Detection (Undirected Acyclic) ===\n";
    std::cout << "Scenario: Verifying if an organizational structure is a tree\n\n";

    Graph<void> org_chart;

    // Build a tree structure: CEO → [VP1, VP2] → [Mgr1, Mgr2, Mgr3]
    auto ceo = org_chart.add_vertex();
    auto vp1 = org_chart.add_vertex();
    auto vp2 = org_chart.add_vertex();
    auto mgr1 = org_chart.add_vertex();
    auto mgr2 = org_chart.add_vertex();
    auto mgr3 = org_chart.add_vertex();

    org_chart.add_edge(ceo, vp1);
    org_chart.add_edge(ceo, vp2);
    org_chart.add_edge(vp1, mgr1);
    org_chart.add_edge(vp1, mgr2);
    org_chart.add_edge(vp2, mgr3);

    if (is_acyclic_undirected(org_chart)) {
        std::cout << "✓ Organizational chart is a tree structure\n";
        std::cout << "  Valid hierarchy with " << org_chart.vertex_count() << " positions\n";
        std::cout << "  and " << org_chart.edge_count() << " reporting lines\n";

        // A tree with n vertices has exactly n-1 edges
        if (org_chart.edge_count() == org_chart.vertex_count() - 1) {
            std::cout << "  Confirmed: n-1 edges for n vertices (tree property)\n";
        }
    } else {
        std::cout << "✗ Organizational chart has reporting cycles!\n";
    }

    // Add a problematic edge creating dual reporting
    std::cout << "\nAdding dual reporting line: Mgr1 → VP2\n";
    org_chart.add_edge(mgr1, vp2);

    if (is_acyclic_undirected(org_chart)) {
        std::cout << "✓ Still acyclic\n";
    } else {
        std::cout << "✗ Creates a cycle - dual reporting causes structural issue!\n";
    }
}

// Example 5: Graph Diameter - Network Analysis
void example_network_diameter() {
    std::cout << "\n=== Example 5: Graph Diameter - Network Analysis ===\n";
    std::cout << "Scenario: Analyzing communication latency in a network\n\n";

    Graph<void> network;

    // Create a network topology: 5 nodes in a chain
    std::vector<size_t> nodes;
    for (int i = 0; i < 5; ++i) {
        nodes.push_back(network.add_vertex());
    }

    // Linear chain: 0 - 1 - 2 - 3 - 4
    network.add_edge(nodes[0], nodes[1], 10.0); // 10ms latency
    network.add_edge(nodes[1], nodes[2], 10.0);
    network.add_edge(nodes[2], nodes[3], 10.0);
    network.add_edge(nodes[3], nodes[4], 10.0);

    double diameter = graph_diameter(network);
    std::cout << "Linear chain network:\n";
    std::cout << "  Diameter (worst-case latency): " << diameter << "ms\n";
    std::cout << "  This is the latency between nodes 0 and 4\n";

    // Add a shortcut
    std::cout << "\nAdding shortcut: 0 → 3 (5ms latency)\n";
    network.add_edge(nodes[0], nodes[3], 5.0);

    diameter = graph_diameter(network);
    std::cout << "  New diameter: " << diameter << "ms\n";
    std::cout << "  Improved worst-case latency!\n";
}

// Example 6: Graph Radius and Center - Finding Optimal Locations
void example_network_center() {
    std::cout << "\n=== Example 6: Graph Radius & Center - Optimal Server Placement ===\n";
    std::cout << "Scenario: Finding optimal location for a central server\n\n";

    Graph<void> network;

    // Create a star topology: center connected to 4 periphery nodes
    auto center_node = network.add_vertex();
    std::vector<size_t> periphery;

    for (int i = 0; i < 4; ++i) {
        auto node = network.add_vertex();
        network.add_edge(center_node, node, 5.0);
        periphery.push_back(node);
    }

    double radius = graph_radius(network);
    auto center_vertices = graph_center(network);

    std::cout << "Star topology (1 center + 4 periphery nodes):\n";
    std::cout << "  Radius (minimum eccentricity): " << radius << "ms\n";
    std::cout << "  Center vertices (optimal server locations): ";
    for (auto v : center_vertices) {
        std::cout << v << " ";
    }
    std::cout << "\n";

    if (center_vertices.size() == 1 && center_vertices[0] == center_node) {
        std::cout << "  ✓ Center node is optimal - minimizes max distance to any client!\n";
    }

    // Now make it a ring
    std::cout << "\nConverting to ring topology (adding edge between periphery nodes):\n";
    for (size_t i = 0; i < periphery.size(); ++i) {
        network.add_edge(periphery[i], periphery[(i + 1) % periphery.size()], 5.0);
    }

    radius = graph_radius(network);
    center_vertices = graph_center(network);

    std::cout << "  New radius: " << radius << "ms\n";
    std::cout << "  New center vertices: ";
    for (auto v : center_vertices) {
        std::cout << v << " ";
    }
    std::cout << "\n";
    std::cout << "  In a ring, multiple nodes may have equal optimality!\n";
}

// Example 7: Combined Analysis - Social Network Metrics
void example_social_network() {
    std::cout << "\n=== Example 7: Combined Analysis - Social Network ===\n";
    std::cout << "Scenario: Analyzing a friendship network\n\n";

    Graph<void> social;

    // Create a small social network
    //     0 - 1 - 2
    //     |   |   |
    //     3 - 4 - 5
    std::vector<size_t> people;
    for (int i = 0; i < 6; ++i) {
        people.push_back(social.add_vertex());
    }

    // First row
    social.add_edge(people[0], people[1]);
    social.add_edge(people[1], people[2]);

    // Second row
    social.add_edge(people[3], people[4]);
    social.add_edge(people[4], people[5]);

    // Vertical connections
    social.add_edge(people[0], people[3]);
    social.add_edge(people[1], people[4]);
    social.add_edge(people[2], people[5]);

    std::cout << "Network stats:\n";
    std::cout << "  Vertices (people): " << social.vertex_count() << "\n";
    std::cout << "  Edges (friendships): " << social.edge_count() << "\n";

    // Check if bipartite (can separate into two groups with no intra-group connections)
    auto bip_result = is_bipartite(social);
    if (bip_result.is_bipartite) {
        std::cout << "  ✓ Network can be split into two groups\n";
        std::cout << "    Group A: ";
        for (const auto &[v, color] : bip_result.coloring) {
            if (color == 0) {
                std::cout << v << " ";
            }
        }
        std::cout << "\n    Group B: ";
        for (const auto &[v, color] : bip_result.coloring) {
            if (color == 1) {
                std::cout << v << " ";
            }
        }
        std::cout << "\n";
    } else {
        std::cout << "  ✗ Network cannot be cleanly split into two groups\n";
    }

    // Check if tree (no cycles)
    if (is_acyclic_undirected(social)) {
        std::cout << "  ✓ Network has no friendship loops (tree structure)\n";
    } else {
        std::cout << "  ✗ Network has friendship loops\n";
    }

    // Network reach metrics
    double diameter = graph_diameter(social);
    double radius = graph_radius(social);
    auto centers = graph_center(social);

    std::cout << "  Diameter (max degrees of separation): " << diameter << "\n";
    std::cout << "  Radius (min max-reach): " << radius << "\n";
    std::cout << "  Most central people (influencers): ";
    for (auto v : centers) {
        std::cout << v << " ";
    }
    std::cout << "\n";
}

int main() {
    std::cout << "Graph Properties Examples\n";
    std::cout << "==========================\n";

    example_bipartite_job_assignment();
    example_bipartite_conflict();
    example_dag_build_system();
    example_tree_detection();
    example_network_diameter();
    example_network_center();
    example_social_network();

    std::cout << "\n=== All examples completed ===\n";
    return 0;
}
