#include "graphix/vertex/algorithms/topological_sort.hpp"
#include "graphix/vertex/graph.hpp"
#include <iostream>
#include <string>

using namespace graphix::vertex;

int main() {
    std::cout << "=== Topological Sort Examples ===\n\n";

    // Example 1: Course Prerequisites
    std::cout << "Example 1: Course Prerequisites\n";
    std::cout << "--------------------------------\n";
    {
        Graph<std::string> courses;

        // Create courses
        auto intro_cs = courses.add_vertex("Intro to CS");
        auto data_struct = courses.add_vertex("Data Structures");
        auto algorithms = courses.add_vertex("Algorithms");
        auto databases = courses.add_vertex("Databases");
        auto web_dev = courses.add_vertex("Web Development");
        auto ai = courses.add_vertex("Artificial Intelligence");

        // Add prerequisites (course -> requires course as prerequisite)
        courses.add_edge(intro_cs, data_struct, 1.0, EdgeType::Directed);
        courses.add_edge(data_struct, algorithms, 1.0, EdgeType::Directed);
        courses.add_edge(data_struct, databases, 1.0, EdgeType::Directed);
        courses.add_edge(intro_cs, web_dev, 1.0, EdgeType::Directed);
        courses.add_edge(algorithms, ai, 1.0, EdgeType::Directed);

        auto result = topological_sort(courses);

        if (result.is_dag) {
            std::cout << "Valid course order found!\n";
            std::cout << "Recommended course sequence:\n";
            for (size_t i = 0; i < result.order.size(); i++) {
                std::cout << "  " << (i + 1) << ". " << courses[result.order[i]] << "\n";
            }
        } else {
            std::cout << "Error: Circular prerequisites detected!\n";
        }
    }

    std::cout << "\n";

    // Example 2: Build System Dependencies
    std::cout << "Example 2: Build System Dependencies\n";
    std::cout << "------------------------------------\n";
    {
        Graph<std::string> build;

        // Source files and build targets
        auto config_h = build.add_vertex("config.h");
        auto utils_h = build.add_vertex("utils.h");
        auto utils_cpp = build.add_vertex("utils.cpp");
        auto utils_o = build.add_vertex("utils.o");
        auto main_cpp = build.add_vertex("main.cpp");
        auto main_o = build.add_vertex("main.o");
        auto app = build.add_vertex("app");

        // Build dependencies (target depends on source)
        build.add_edge(config_h, utils_o, 1.0, EdgeType::Directed);
        build.add_edge(utils_h, utils_o, 1.0, EdgeType::Directed);
        build.add_edge(utils_cpp, utils_o, 1.0, EdgeType::Directed);
        build.add_edge(config_h, main_o, 1.0, EdgeType::Directed);
        build.add_edge(utils_h, main_o, 1.0, EdgeType::Directed);
        build.add_edge(main_cpp, main_o, 1.0, EdgeType::Directed);
        build.add_edge(main_o, app, 1.0, EdgeType::Directed);
        build.add_edge(utils_o, app, 1.0, EdgeType::Directed);

        auto result = topological_sort(build);

        if (result.is_dag) {
            std::cout << "Build order:\n";
            for (size_t i = 0; i < result.order.size(); i++) {
                std::cout << "  " << (i + 1) << ". " << build[result.order[i]] << "\n";
            }
        }
    }

    std::cout << "\n";

    // Example 3: Task Scheduling
    std::cout << "Example 3: Project Task Dependencies\n";
    std::cout << "-------------------------------------\n";
    {
        Graph<std::string> project;

        auto requirements = project.add_vertex("Gather Requirements");
        auto design = project.add_vertex("Design Architecture");
        auto setup_env = project.add_vertex("Setup Environment");
        auto backend = project.add_vertex("Develop Backend");
        auto frontend = project.add_vertex("Develop Frontend");
        auto testing = project.add_vertex("Integration Testing");
        auto deployment = project.add_vertex("Deploy to Production");

        // Task dependencies
        project.add_edge(requirements, design, 1.0, EdgeType::Directed);
        project.add_edge(design, backend, 1.0, EdgeType::Directed);
        project.add_edge(design, frontend, 1.0, EdgeType::Directed);
        project.add_edge(requirements, setup_env, 1.0, EdgeType::Directed);
        project.add_edge(setup_env, backend, 1.0, EdgeType::Directed);
        project.add_edge(setup_env, frontend, 1.0, EdgeType::Directed);
        project.add_edge(backend, testing, 1.0, EdgeType::Directed);
        project.add_edge(frontend, testing, 1.0, EdgeType::Directed);
        project.add_edge(testing, deployment, 1.0, EdgeType::Directed);

        auto result = topological_sort(project);

        if (result.is_dag) {
            std::cout << "Task execution order:\n";
            for (size_t i = 0; i < result.order.size(); i++) {
                std::cout << "  " << (i + 1) << ". " << project[result.order[i]] << "\n";
            }
        }
    }

    std::cout << "\n";

    // Example 4: Cycle Detection
    std::cout << "Example 4: Detecting Circular Dependencies\n";
    std::cout << "-------------------------------------------\n";
    {
        Graph<std::string> modules;

        auto mod_a = modules.add_vertex("Module A");
        auto mod_b = modules.add_vertex("Module B");
        auto mod_c = modules.add_vertex("Module C");

        // Create circular dependency: A -> B -> C -> A
        modules.add_edge(mod_a, mod_b, 1.0, EdgeType::Directed);
        modules.add_edge(mod_b, mod_c, 1.0, EdgeType::Directed);
        modules.add_edge(mod_c, mod_a, 1.0, EdgeType::Directed);

        auto result = topological_sort(modules);

        if (!result.is_dag) {
            std::cout << "Circular dependency detected!\n";
            std::cout << "Module dependencies cannot be satisfied.\n";
            std::cout << "This indicates a design problem that needs to be fixed.\n";
        } else {
            std::cout << "No circular dependencies found.\n";
        }
    }

    std::cout << "\n";

    // Example 5: Comparing Kahn's vs DFS algorithms
    std::cout << "Example 5: Comparing Algorithms (Kahn's vs DFS)\n";
    std::cout << "------------------------------------------------\n";
    {
        Graph<std::string> dag;

        auto a = dag.add_vertex("A");
        auto b = dag.add_vertex("B");
        auto c = dag.add_vertex("C");
        auto d = dag.add_vertex("D");
        auto e = dag.add_vertex("E");

        dag.add_edge(a, b, 1.0, EdgeType::Directed);
        dag.add_edge(a, c, 1.0, EdgeType::Directed);
        dag.add_edge(b, d, 1.0, EdgeType::Directed);
        dag.add_edge(c, d, 1.0, EdgeType::Directed);
        dag.add_edge(d, e, 1.0, EdgeType::Directed);

        auto kahn_result = topological_sort(dag);
        auto dfs_result = topological_sort_dfs(dag);

        std::cout << "Kahn's algorithm order: ";
        for (auto v : kahn_result.order) {
            std::cout << dag[v] << " ";
        }
        std::cout << "\n";

        std::cout << "DFS algorithm order:    ";
        for (auto v : dfs_result.order) {
            std::cout << dag[v] << " ";
        }
        std::cout << "\n";

        std::cout << "\nBoth orderings are valid! The order may differ,\n";
        std::cout << "but both satisfy all edge constraints.\n";
    }

    std::cout << "\n";

    // Example 6: Real-world Application - Package Manager
    std::cout << "Example 6: Package Manager Dependency Resolution\n";
    std::cout << "-------------------------------------------------\n";
    {
        Graph<std::string> packages;

        auto libssl = packages.add_vertex("libssl");
        auto libcrypto = packages.add_vertex("libcrypto");
        auto zlib = packages.add_vertex("zlib");
        auto curl = packages.add_vertex("curl");
        auto git = packages.add_vertex("git");
        auto python = packages.add_vertex("python");
        auto pip = packages.add_vertex("pip");

        // Package dependencies
        packages.add_edge(libcrypto, libssl, 1.0, EdgeType::Directed);
        packages.add_edge(zlib, libssl, 1.0, EdgeType::Directed);
        packages.add_edge(libssl, curl, 1.0, EdgeType::Directed);
        packages.add_edge(curl, git, 1.0, EdgeType::Directed);
        packages.add_edge(zlib, python, 1.0, EdgeType::Directed);
        packages.add_edge(python, pip, 1.0, EdgeType::Directed);

        auto result = topological_sort(packages);

        if (result.is_dag) {
            std::cout << "Package installation order:\n";
            for (size_t i = 0; i < result.order.size(); i++) {
                std::cout << "  " << (i + 1) << ". Install " << packages[result.order[i]] << "\n";
            }
            std::cout << "\nThis ensures all dependencies are installed before packages that need them.\n";
        }
    }

    return 0;
}
