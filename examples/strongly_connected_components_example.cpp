#include "graphix/vertex/algorithms/strongly_connected_components.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>
#include <string>

using namespace graphix::vertex;

void print_sccs(const SCCResult &result, const Graph<std::string> &g) {
    std::cout << "  Found " << result.num_components << " strongly connected component(s):\n";
    for (size_t i = 0; i < result.components.size(); i++) {
        std::cout << "  SCC " << (i + 1) << ": {";
        for (size_t j = 0; j < result.components[i].size(); j++) {
            std::cout << g[result.components[i][j]];
            if (j < result.components[i].size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "}\n";
    }
}

void print_sccs_void(const SCCResult &result) {
    std::cout << "  Found " << result.num_components << " strongly connected component(s):\n";
    for (size_t i = 0; i < result.components.size(); i++) {
        std::cout << "  SCC " << (i + 1) << ": {";
        for (size_t j = 0; j < result.components[i].size(); j++) {
            std::cout << "v" << result.components[i][j];
            if (j < result.components[i].size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "}\n";
    }
}

int main() {
    std::cout << "=== Strongly Connected Components Examples ===\n\n";

    // Example 1: Web Pages and Links
    std::cout << "Example 1: Web Page Link Analysis\n";
    std::cout << "----------------------------------\n";
    {
        Graph<std::string> web;

        auto home = web.add_vertex("Home");
        auto about = web.add_vertex("About");
        auto products = web.add_vertex("Products");
        auto contact = web.add_vertex("Contact");
        auto blog = web.add_vertex("Blog");

        // Create link structure
        web.add_edge(home, about, 1.0, EdgeType::Directed);
        web.add_edge(about, home, 1.0, EdgeType::Directed); // Home <-> About (SCC)
        web.add_edge(home, products, 1.0, EdgeType::Directed);
        web.add_edge(products, contact, 1.0, EdgeType::Directed);
        web.add_edge(contact, products, 1.0, EdgeType::Directed); // Products <-> Contact (SCC)
        web.add_edge(about, blog, 1.0, EdgeType::Directed);       // Blog is separate

        auto result = strongly_connected_components(web);
        print_sccs(result, web);

        std::cout << "\nPages in the same SCC can reach each other bidirectionally.\n";
        std::cout << "This helps identify tightly coupled page groups!\n";
    }

    std::cout << "\n";

    // Example 2: Call Graph Analysis
    std::cout << "Example 2: Function Call Graph (Mutual Recursion)\n";
    std::cout << "--------------------------------------------------\n";
    {
        Graph<std::string> calls;

        auto main_fn = calls.add_vertex("main()");
        auto foo = calls.add_vertex("foo()");
        auto bar = calls.add_vertex("bar()");
        auto helper = calls.add_vertex("helper()");

        // Call relationships
        calls.add_edge(main_fn, foo, 1.0, EdgeType::Directed);
        calls.add_edge(foo, bar, 1.0, EdgeType::Directed);
        calls.add_edge(bar, foo, 1.0, EdgeType::Directed); // foo <-> bar (mutual recursion)
        calls.add_edge(bar, helper, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(calls);
        print_sccs(result, calls);

        std::cout << "\nMutually recursive functions form an SCC.\n";
        std::cout << "This helps detect recursion patterns in code analysis!\n";
    }

    std::cout << "\n";

    // Example 3: Task Dependencies with Cycles
    std::cout << "Example 3: Task Dependency Analysis\n";
    std::cout << "------------------------------------\n";
    {
        Graph<std::string> tasks;

        auto task_a = tasks.add_vertex("Task A");
        auto task_b = tasks.add_vertex("Task B");
        auto task_c = tasks.add_vertex("Task C");
        auto task_d = tasks.add_vertex("Task D");
        auto task_e = tasks.add_vertex("Task E");

        // Dependencies (A->B means A depends on B)
        tasks.add_edge(task_a, task_b, 1.0, EdgeType::Directed);
        tasks.add_edge(task_b, task_c, 1.0, EdgeType::Directed);
        tasks.add_edge(task_c, task_b, 1.0, EdgeType::Directed); // B <-> C (circular dependency!)
        tasks.add_edge(task_c, task_d, 1.0, EdgeType::Directed);
        tasks.add_edge(task_a, task_e, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(tasks);
        print_sccs(result, tasks);

        std::cout << "\nTasks in the same SCC have circular dependencies.\n";
        std::cout << "They must be executed together or refactored!\n";
    }

    std::cout << "\n";

    // Example 4: Social Network Cliques
    std::cout << "Example 4: Social Network Analysis\n";
    std::cout << "-----------------------------------\n";
    {
        Graph<std::string> social;

        auto alice = social.add_vertex("Alice");
        auto bob = social.add_vertex("Bob");
        auto carol = social.add_vertex("Carol");
        auto dave = social.add_vertex("Dave");
        auto eve = social.add_vertex("Eve");

        // Mutual friendships (using directed for "follows" relationship)
        social.add_edge(alice, bob, 1.0, EdgeType::Directed);
        social.add_edge(bob, alice, 1.0, EdgeType::Directed);
        social.add_edge(bob, carol, 1.0, EdgeType::Directed);
        social.add_edge(carol, bob, 1.0, EdgeType::Directed);
        social.add_edge(alice, carol, 1.0, EdgeType::Directed);
        social.add_edge(carol, alice, 1.0, EdgeType::Directed); // Alice-Bob-Carol form tight group

        social.add_edge(dave, eve, 1.0, EdgeType::Directed);
        social.add_edge(eve, dave, 1.0, EdgeType::Directed); // Dave-Eve are separate pair

        auto result = strongly_connected_components(social);
        print_sccs(result, social);

        std::cout << "\nStrongly connected users can all reach each other.\n";
        std::cout << "These are your friend groups/cliques!\n";
    }

    std::cout << "\n";

    // Example 5: Compiler Optimization (Variable Dependencies)
    std::cout << "Example 5: Variable Dependency in Compiler\n";
    std::cout << "-------------------------------------------\n";
    {
        Graph<std::string> vars;

        auto x = vars.add_vertex("x");
        auto y = vars.add_vertex("y");
        auto z = vars.add_vertex("z");
        auto w = vars.add_vertex("w");

        // Variable dependencies (x->y means x's value depends on y)
        vars.add_edge(x, y, 1.0, EdgeType::Directed);
        vars.add_edge(y, z, 1.0, EdgeType::Directed);
        vars.add_edge(z, x, 1.0, EdgeType::Directed); // x->y->z->x (cyclic dependency)
        vars.add_edge(z, w, 1.0, EdgeType::Directed);

        auto result = strongly_connected_components(vars);
        print_sccs(result, vars);

        std::cout << "\nVariables in same SCC must be computed together.\n";
        std::cout << "Cannot be optimized independently!\n";
    }

    std::cout << "\n";

    // Example 6: DAG Condensation
    std::cout << "Example 6: Graph Condensation (Converting to DAG)\n";
    std::cout << "---------------------------------------------------\n";
    {
        Graph<void> g;

        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();
        auto v3 = g.add_vertex();
        auto v4 = g.add_vertex();

        // Create structure: 0->1<->2->3<->4
        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v1, 1.0, EdgeType::Directed); // 1<->2
        g.add_edge(v2, v3, 1.0, EdgeType::Directed);
        g.add_edge(v3, v4, 1.0, EdgeType::Directed);
        g.add_edge(v4, v3, 1.0, EdgeType::Directed); // 3<->4

        std::cout << "Original graph has cycles.\n";
        auto result = strongly_connected_components(g);
        print_sccs_void(result);

        std::cout << "\nBy treating each SCC as a single node,\n";
        std::cout << "we can create a DAG from a cyclic graph!\n";
        std::cout << "Condensed DAG: SCC1 -> SCC2 -> SCC3\n";
    }

    std::cout << "\n";

    // Example 7: Using Helper Functions
    std::cout << "Example 7: Helper Function Demonstrations\n";
    std::cout << "------------------------------------------\n";
    {
        Graph<void> g;
        auto v0 = g.add_vertex();
        auto v1 = g.add_vertex();
        auto v2 = g.add_vertex();

        g.add_edge(v0, v1, 1.0, EdgeType::Directed);
        g.add_edge(v1, v2, 1.0, EdgeType::Directed);
        g.add_edge(v2, v0, 1.0, EdgeType::Directed);

        std::cout << "Graph: v0 -> v1 -> v2 -> v0 (triangle)\n\n";

        // is_strongly_connected
        if (is_strongly_connected(g)) {
            std::cout << "✓ Graph is strongly connected!\n";
        }

        // in_same_scc
        if (in_same_scc(g, v0, v2)) {
            std::cout << "✓ v0 and v2 are in the same SCC\n";
        }

        // largest_scc_size
        std::cout << "✓ Largest SCC size: " << largest_scc_size(g) << "\n";

        // get_component_map
        auto comp_map = get_component_map(g);
        std::cout << "✓ Component map created: " << comp_map.size() << " vertices mapped\n";
    }

    std::cout << "\n";

    // Example 8: Course Registration (Prerequisite Cycles)
    std::cout << "Example 8: Detecting Circular Prerequisites\n";
    std::cout << "---------------------------------------------\n";
    {
        Graph<std::string> courses;

        auto cs101 = courses.add_vertex("CS101");
        auto cs201 = courses.add_vertex("CS201");
        auto cs301 = courses.add_vertex("CS301");
        auto math101 = courses.add_vertex("Math101");

        // Prerequisites
        courses.add_edge(cs201, cs101, 1.0, EdgeType::Directed);   // CS201 requires CS101
        courses.add_edge(cs301, cs201, 1.0, EdgeType::Directed);   // CS301 requires CS201
        courses.add_edge(cs101, cs301, 1.0, EdgeType::Directed);   // CS101 requires CS301! (ERROR)
        courses.add_edge(cs201, math101, 1.0, EdgeType::Directed); // CS201 requires Math101

        auto result = strongly_connected_components(courses);
        print_sccs(result, courses);

        std::cout << "\n❌ ERROR: Courses CS101, CS201, and CS301 form a circular prerequisite!\n";
        std::cout << "This is impossible to satisfy and must be fixed.\n";
    }

    std::cout << "\n";

    // Example 9: Network Reachability
    std::cout << "Example 9: Network Reachability Analysis\n";
    std::cout << "-----------------------------------------\n";
    {
        Graph<std::string> network;

        auto router1 = network.add_vertex("Router1");
        auto router2 = network.add_vertex("Router2");
        auto router3 = network.add_vertex("Router3");
        auto router4 = network.add_vertex("Router4");

        // Routing paths
        network.add_edge(router1, router2, 1.0, EdgeType::Directed);
        network.add_edge(router2, router3, 1.0, EdgeType::Directed);
        network.add_edge(router3, router1, 1.0, EdgeType::Directed); // 1-2-3 can all reach each other
        network.add_edge(router3, router4, 1.0, EdgeType::Directed); // One-way to router4

        auto result = strongly_connected_components(network);
        print_sccs(result, network);

        std::cout << "\nRouters in the same SCC have bidirectional reachability.\n";
        std::cout << "Router4 can be reached but cannot reach back!\n";
    }

    std::cout << "\n";

    // Example 10: Performance Test
    std::cout << "Example 10: Large Graph Performance\n";
    std::cout << "------------------------------------\n";
    {
        Graph<void> large;
        std::vector<size_t> vertices;

        // Create 50 vertices
        for (int i = 0; i < 50; i++) {
            vertices.push_back(large.add_vertex());
        }

        // Create 5 SCCs of 10 vertices each
        for (int scc = 0; scc < 5; scc++) {
            int start = scc * 10;
            // Create cycle within each SCC
            for (int i = 0; i < 10; i++) {
                int next = (i + 1) % 10;
                large.add_edge(vertices[start + i], vertices[start + next], 1.0, EdgeType::Directed);
            }
        }

        auto result = strongly_connected_components(large);

        std::cout << "Graph with 50 vertices organized into cycles:\n";
        std::cout << "  Found " << result.num_components << " SCCs\n";
        std::cout << "  Each SCC has 10 vertices\n";
        std::cout << "\nTarjan's algorithm handles large graphs efficiently!\n";
        std::cout << "Time complexity: O(V + E)\n";
    }

    return 0;
}
