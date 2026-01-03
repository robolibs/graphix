#include "graphix/vertex/algorithms/astar.hpp"
#include "graphix/vertex/graph.hpp"
#include <cmath>
#include <iostream>
#include <string>

// A* pathfinding example: Demonstrating heuristic-guided search

using namespace graphix::vertex;
using namespace graphix::vertex::algorithms;

// Example 1: Grid-based pathfinding (like video game navigation)
void grid_pathfinding_example() {
    std::cout << "=== Example 1: Grid-based Pathfinding ===" << std::endl;
    std::cout << std::endl;

    // Create a 5x5 grid graph
    // Grid layout (vertex IDs):
    //  0  1  2  3  4
    //  5  6  7  8  9
    // 10 11 12 13 14
    // 15 16 17 18 19
    // 20 21 22 23 24

    Graph<void> grid;
    const int width = 5;
    const int height = 5;

    // Add vertices
    for (int i = 0; i < width * height; ++i) {
        grid.add_vertex();
    }

    // Add edges (4-directional movement with cost 1.0)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int current = y * width + x;

            // Right edge
            if (x < width - 1) {
                grid.add_edge(current, current + 1, 1.0);
            }
            // Down edge
            if (y < height - 1) {
                grid.add_edge(current, current + width, 1.0);
            }
        }
    }

    // Remove some edges to create obstacles (walls)
    std::cout << "Creating obstacles in the grid..." << std::endl;
    // Create a vertical wall from (2,1) to (2,3)
    grid.remove_edge(7, 12);  // Block (2,1) to (2,2)
    grid.remove_edge(12, 17); // Block (2,2) to (2,3)

    size_t start = 0; // Top-left corner
    size_t goal = 24; // Bottom-right corner

    // Manhattan distance heuristic for grid
    auto grid_heuristic = [width](size_t a, size_t b) { return manhattan_distance(a, b, width); };

    std::cout << "Finding path from top-left (0) to bottom-right (24)..." << std::endl;
    auto result = astar(grid, start, goal, grid_heuristic);

    if (result.found) {
        std::cout << "  Path found!" << std::endl;
        std::cout << "  Distance: " << result.distance << std::endl;
        std::cout << "  Nodes explored: " << result.nodes_explored << std::endl;
        std::cout << "  Path: ";
        for (size_t i = 0; i < result.path.size(); ++i) {
            std::cout << result.path[i];
            if (i < result.path.size() - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;

        // Visualize the path
        std::cout << "\n  Grid visualization (X = path, # = obstacle):" << std::endl;
        for (int y = 0; y < height; ++y) {
            std::cout << "    ";
            for (int x = 0; x < width; ++x) {
                int id = y * width + x;
                bool in_path = false;
                for (auto v : result.path) {
                    if (v == static_cast<size_t>(id)) {
                        in_path = true;
                        break;
                    }
                }
                // Check if it's an obstacle (simplified check)
                bool is_obstacle = (x == 2 && (y == 1 || y == 2));

                if (in_path)
                    std::cout << "X ";
                else if (is_obstacle)
                    std::cout << "# ";
                else
                    std::cout << ". ";
            }
            std::cout << std::endl;
        }
    }
    std::cout << std::endl;
}

// Example 2: City navigation with coordinates
struct City {
    std::string name;
    double x, y; // Coordinates

    City(const std::string &n = "", double x_coord = 0.0, double y_coord = 0.0) : name(n), x(x_coord), y(y_coord) {}
};

void city_navigation_example() {
    std::cout << "=== Example 2: City Navigation with Coordinates ===" << std::endl;
    std::cout << std::endl;

    // Create a graph with cities at specific coordinates
    Graph<City> map;

    // Add cities with approximate coordinates (simplified)
    auto seattle = map.add_vertex(City("Seattle", 0.0, 100.0));
    auto portland = map.add_vertex(City("Portland", 0.0, 90.0));
    auto sf = map.add_vertex(City("San Francisco", 0.0, 70.0));
    auto la = map.add_vertex(City("Los Angeles", 10.0, 60.0));
    auto vegas = map.add_vertex(City("Las Vegas", 20.0, 60.0));
    auto phoenix = map.add_vertex(City("Phoenix", 25.0, 50.0));
    auto denver = map.add_vertex(City("Denver", 40.0, 70.0));

    // Add roads with actual distances
    map.add_edge(seattle, portland, 1.7);
    map.add_edge(portland, sf, 6.4);
    map.add_edge(sf, la, 3.8);
    map.add_edge(la, vegas, 2.7);
    map.add_edge(la, phoenix, 3.7);
    map.add_edge(vegas, phoenix, 2.9);
    map.add_edge(vegas, denver, 7.5);
    map.add_edge(phoenix, denver, 6.0);

    // Euclidean distance heuristic based on coordinates
    auto coord_heuristic = [&map](size_t a, size_t b) {
        struct Point {
            double x, y;
        };
        Point pa{map[a].x, map[a].y};
        Point pb{map[b].x, map[b].y};
        return euclidean_distance(pa, pb);
    };

    std::cout << "Finding path from Seattle to Denver..." << std::endl;
    auto result = astar(map, seattle, denver, coord_heuristic);

    if (result.found) {
        std::cout << "  Distance: " << result.distance << " (units)" << std::endl;
        std::cout << "  Nodes explored: " << result.nodes_explored << std::endl;
        std::cout << "  Route: ";
        for (size_t i = 0; i < result.path.size(); ++i) {
            std::cout << map[result.path[i]].name;
            if (i < result.path.size() - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // Compare A* with Dijkstra (zero heuristic)
    std::cout << "Comparing A* vs Dijkstra (zero heuristic):" << std::endl;

    auto dijkstra_result = astar_dijkstra(map, seattle, denver);
    std::cout << "  Dijkstra nodes explored: " << dijkstra_result.nodes_explored << std::endl;
    std::cout << "  A* nodes explored: " << result.nodes_explored << std::endl;

    if (result.nodes_explored < dijkstra_result.nodes_explored) {
        double improvement =
            100.0 * (1.0 - static_cast<double>(result.nodes_explored) / dijkstra_result.nodes_explored);
        std::cout << "  A* explored " << improvement << "% fewer nodes!" << std::endl;
    }

    std::cout << std::endl;
}

// Example 3: Maze solving
void maze_solving_example() {
    std::cout << "=== Example 3: Maze Solving ===" << std::endl;
    std::cout << std::endl;

    // Create a maze graph (7x7 grid)
    Graph<void> maze;
    const int maze_size = 7;

    for (int i = 0; i < maze_size * maze_size; ++i) {
        maze.add_vertex();
    }

    // Define the maze structure (1 = wall, 0 = passage)
    // clang-format off
    int maze_layout[7][7] = {
        {0, 0, 0, 1, 0, 0, 0},
        {1, 1, 0, 1, 0, 1, 0},
        {0, 0, 0, 0, 0, 1, 0},
        {0, 1, 1, 1, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0},
        {1, 1, 0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0, 0, 0}
    };
    // clang-format on

    // Add edges based on maze layout
    for (int y = 0; y < maze_size; ++y) {
        for (int x = 0; x < maze_size; ++x) {
            if (maze_layout[y][x] == 1)
                continue; // Skip walls

            int current = y * maze_size + x;

            // Right edge
            if (x < maze_size - 1 && maze_layout[y][x + 1] == 0) {
                maze.add_edge(current, current + 1, 1.0);
            }
            // Down edge
            if (y < maze_size - 1 && maze_layout[y + 1][x] == 0) {
                maze.add_edge(current, current + maze_size, 1.0);
            }
            // Left edge
            if (x > 0 && maze_layout[y][x - 1] == 0) {
                maze.add_edge(current, current - 1, 1.0);
            }
            // Up edge
            if (y > 0 && maze_layout[y - 1][x] == 0) {
                maze.add_edge(current, current - maze_size, 1.0);
            }
        }
    }

    size_t maze_start = 0;                        // Top-left
    size_t maze_goal = maze_size * maze_size - 1; // Bottom-right

    auto maze_heuristic = [maze_size](size_t a, size_t b) { return manhattan_distance(a, b, maze_size); };

    std::cout << "Solving maze from top-left to bottom-right..." << std::endl;
    auto result = astar(maze, maze_start, maze_goal, maze_heuristic);

    if (result.found) {
        std::cout << "  Solution found!" << std::endl;
        std::cout << "  Path length: " << result.distance << std::endl;
        std::cout << "  Nodes explored: " << result.nodes_explored << " out of " << maze_size * maze_size << std::endl;

        std::cout << "\n  Maze solution (S = start, G = goal, * = path, # = wall):" << std::endl;
        for (int y = 0; y < maze_size; ++y) {
            std::cout << "    ";
            for (int x = 0; x < maze_size; ++x) {
                int id = y * maze_size + x;

                bool in_path = false;
                for (auto v : result.path) {
                    if (v == static_cast<size_t>(id)) {
                        in_path = true;
                        break;
                    }
                }

                if (id == static_cast<int>(maze_start))
                    std::cout << "S ";
                else if (id == static_cast<int>(maze_goal))
                    std::cout << "G ";
                else if (maze_layout[y][x] == 1)
                    std::cout << "# ";
                else if (in_path)
                    std::cout << "* ";
                else
                    std::cout << ". ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cout << "  No solution found!" << std::endl;
    }
    std::cout << std::endl;
}

// Example 4: Robot navigation with obstacles
void robot_navigation_example() {
    std::cout << "=== Example 4: Robot Navigation ===" << std::endl;
    std::cout << std::endl;

    // Create a warehouse layout (4x4 grid with some blocked areas)
    Graph<void> warehouse;
    const int w_size = 4;

    for (int i = 0; i < w_size * w_size; ++i) {
        warehouse.add_vertex();
    }

    // Add edges with varying costs (some paths are slower)
    for (int y = 0; y < w_size; ++y) {
        for (int x = 0; x < w_size; ++x) {
            int current = y * w_size + x;

            // Right edge
            if (x < w_size - 1) {
                // Higher cost for y=1 (conveyor belt area)
                double cost = (y == 1) ? 3.0 : 1.0;
                warehouse.add_edge(current, current + 1, cost);
            }
            // Down edge
            if (y < w_size - 1) {
                double cost = (y == 1) ? 3.0 : 1.0;
                warehouse.add_edge(current, current + w_size, cost);
            }
        }
    }

    size_t robot_start = 0; // Top-left
    size_t robot_goal = 15; // Bottom-right

    auto robot_heuristic = [w_size](size_t a, size_t b) { return manhattan_distance(a, b, w_size); };

    std::cout << "Planning robot path (avoiding slow conveyor belt areas)..." << std::endl;
    auto result = astar(warehouse, robot_start, robot_goal, robot_heuristic);

    if (result.found) {
        std::cout << "  Path cost: " << result.distance << std::endl;
        std::cout << "  Path: ";
        for (size_t i = 0; i < result.path.size(); ++i) {
            std::cout << result.path[i];
            if (i < result.path.size() - 1)
                std::cout << " -> ";
        }
        std::cout << std::endl;
        std::cout << "  (Robot avoids slow conveyor belt area at y=1)" << std::endl;
    }
    std::cout << std::endl;
}

int main() {
    std::cout << "\n╔════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║        A* Pathfinding Algorithm Examples              ║" << std::endl;
    std::cout << "╚════════════════════════════════════════════════════════╝\n" << std::endl;

    grid_pathfinding_example();
    city_navigation_example();
    maze_solving_example();
    robot_navigation_example();

    std::cout << "=== Summary ===" << std::endl;
    std::cout << "A* is a best-first search algorithm that uses:" << std::endl;
    std::cout << "  - g(n): actual cost from start to node n" << std::endl;
    std::cout << "  - h(n): heuristic estimate from n to goal" << std::endl;
    std::cout << "  - f(n) = g(n) + h(n): total estimated cost" << std::endl;
    std::cout << "\nWith a good heuristic, A* explores fewer nodes than Dijkstra!" << std::endl;
    std::cout << "\nCommon heuristics:" << std::endl;
    std::cout << "  - Manhattan distance: for grid-based graphs" << std::endl;
    std::cout << "  - Euclidean distance: for coordinate-based graphs" << std::endl;
    std::cout << "  - Zero heuristic: equivalent to Dijkstra's algorithm" << std::endl;

    return 0;
}
