<img align="right" width="26%" src="./misc/logo.png">

# Graphix

A modern C++20 library unifying traditional graph algorithms with factor graphs for optimization and probabilistic inference.

## Development Status

See [TODO.md](./TODO.md) for the complete development plan and current progress.

## Overview

Graphix provides two complementary graph paradigms in a single, cohesive library:

**Vertex Graphs** - Traditional graph data structures with comprehensive algorithms for pathfinding, connectivity analysis, and graph transformations. The API follows Boost.Graph conventions while leveraging modern C++20 features for cleaner, more expressive code.

**Factor Graphs** - Probabilistic graphical models for nonlinear least-squares optimization. Built on proper Lie group mathematics (SE2/SO2) via the optinum library, Graphix enables robust SLAM, sensor fusion, and state estimation with Gauss-Newton, Levenberg-Marquardt, and gradient descent optimizers.

The library is header-only, dependency-light, and designed for robotics applications where both graph algorithms and optimization are needed in a unified framework.

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              GRAPHIX                                     │
├─────────────────────────────────┬───────────────────────────────────────┤
│         VERTEX GRAPHS           │           FACTOR GRAPHS               │
│                                 │                                       │
│  ┌───────────────────────────┐  │  ┌─────────────────────────────────┐  │
│  │      Graph<Property>      │  │  │     Graph<NonlinearFactor>      │  │
│  │  - add_vertex/add_edge    │  │  │  - SE2PriorFactor               │  │
│  │  - neighbors/degree       │  │  │  - SE2BetweenFactor             │  │
│  │  - directed/undirected    │  │  │  - PriorFactor/BetweenFactor    │  │
│  └───────────────────────────┘  │  └─────────────────────────────────┘  │
│              │                  │                  │                    │
│  ┌───────────▼───────────────┐  │  ┌───────────────▼─────────────────┐  │
│  │       Algorithms          │  │  │         Optimizers              │  │
│  │  - Dijkstra, A*, BFS/DFS  │  │  │  - GaussNewtonOptimizer         │  │
│  │  - Bellman-Ford, MST      │  │  │  - LevenbergMarquardtOptimizer  │  │
│  │  - SCC, Topological Sort  │  │  │  - GradientDescentOptimizer     │  │
│  │  - Cycle Detection        │  │  │  - Robust Loss Functions        │  │
│  │  - Centrality Measures    │  │  │    (Huber, Cauchy, Tukey)       │  │
│  └───────────────────────────┘  │  └─────────────────────────────────┘  │
│              │                  │                  │                    │
│  ┌───────────▼───────────────┐  │  ┌───────────────▼─────────────────┐  │
│  │    Graph Utilities        │  │  │      Linearization              │  │
│  │  - PropertyMap            │  │  │  - GaussianFactor               │  │
│  │  - Views & Transforms     │  │  │  - Jacobian computation         │  │
│  │  - Serialization          │  │  │  - Values container             │  │
│  └───────────────────────────┘  │  └─────────────────────────────────┘  │
└─────────────────────────────────┴───────────────────────────────────────┘
                                  │
                    ┌─────────────▼─────────────┐
                    │     Dependencies          │
                    │  - datapod (containers)   │
                    │  - optinum (math/Lie)     │
                    └───────────────────────────┘
```

## Installation

### Quick Start (CMake FetchContent)

```cmake
include(FetchContent)
FetchContent_Declare(
  graphix
  GIT_REPOSITORY https://github.com/robolibs/graphix
  GIT_TAG main
)
FetchContent_MakeAvailable(graphix)

target_link_libraries(your_target PRIVATE graphix)
```

### Recommended: XMake

[XMake](https://xmake.io/) is a modern, fast, and cross-platform build system.

**Install XMake:**
```bash
curl -fsSL https://xmake.io/shget.text | bash
```

**Add to your xmake.lua:**
```lua
add_requires("graphix")

target("your_target")
    set_kind("binary")
    add_packages("graphix")
    add_files("src/*.cpp")
```

**Build:**
```bash
xmake
xmake run
```

### Complete Development Environment (Nix + Direnv + Devbox)

For the ultimate reproducible development environment:

**1. Install Nix (package manager from NixOS):**
```bash
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
```

**2. Install direnv (automatic environment switching):**
```bash
sudo apt install direnv
eval "$(direnv hook bash)"  # Add to ~/.bashrc
```

**3. Install Devbox (Nix-powered development environments):**
```bash
curl -fsSL https://get.jetpack.io/devbox | bash
```

**4. Use the environment:**
```bash
cd graphix
direnv allow
xmake
```

## Usage

### Vertex Graphs - Basic Usage

```cpp
#include "graphix/vertex/graph.hpp"
#include "graphix/vertex/algorithms/dijkstra.hpp"

using namespace graphix::vertex;

// Create a weighted graph
Graph<void> g;
auto a = g.add_vertex();
auto b = g.add_vertex();
auto c = g.add_vertex();

g.add_edge(a, b, 1.0);
g.add_edge(b, c, 2.0);
g.add_edge(a, c, 4.0);

// Find shortest path
auto [distances, predecessors] = dijkstra(g, a);
// distances[c] == 3.0 (via b)
```

### Vertex Graphs - With Properties

```cpp
#include "graphix/vertex/graph.hpp"

struct City {
    std::string name;
    int population;
};

Graph<City> cities;
auto nyc = cities.add_vertex({"New York", 8336817});
auto la = cities.add_vertex({"Los Angeles", 3979576});
cities.add_edge(nyc, la, 2790.0);  // Distance in miles

std::cout << cities[nyc].name << std::endl;  // "New York"
```

### Factor Graphs - 2D SLAM with SE2

```cpp
#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/se2_prior_factor.hpp"
#include "graphix/factor/nonlinear/se2_between_factor.hpp"
#include "graphix/factor/nonlinear/levenberg_marquardt.hpp"
#include "graphix/kernel.hpp"

using namespace graphix;
using namespace graphix::factor;

// Create factor graph
Graph<NonlinearFactor> graph;

// Prior on first pose (anchor)
SE2d prior(0, 0, 0);  // theta=0, x=0, y=0
Vec3d prior_sigma{0.1, 0.1, 0.01};
graph.add(std::make_shared<SE2PriorFactor>(X(0), prior, prior_sigma));

// Odometry measurements
SE2d odom(0, 1.0, 0);  // Move 1m forward
Vec3d odom_sigma{0.1, 0.1, 0.05};
graph.add(std::make_shared<SE2BetweenFactor>(X(0), X(1), odom, odom_sigma));
graph.add(std::make_shared<SE2BetweenFactor>(X(1), X(2), odom, odom_sigma));

// Initial estimates
Values initial;
initial.insert<SE2d>(X(0), SE2d(0, 0, 0));
initial.insert<SE2d>(X(1), SE2d(0, 1.0, 0));
initial.insert<SE2d>(X(2), SE2d(0, 2.0, 0));

// Optimize with Levenberg-Marquardt
LevenbergMarquardtOptimizer optimizer;
auto result = optimizer.optimize(graph, initial);

SE2d optimized_pose = result.values.at<SE2d>(X(2));
```

### Robust Optimization with Loss Functions

```cpp
#include "graphix/factor/loss_function.hpp"

// Create robust factors that downweight outliers
auto huber_loss = std::make_shared<HuberLoss>(1.0);
auto cauchy_loss = std::make_shared<CauchyLoss>(1.0);

// Apply to factors for outlier rejection
auto robust_factor = std::make_shared<SE2BetweenFactor>(
    X(0), X(1), measurement, sigma);
robust_factor->set_loss_function(huber_loss);
```

## Features

### Vertex Graph Algorithms

Comprehensive graph algorithms following Boost.Graph conventions:

- **Shortest Paths**
  - `dijkstra()` - Single-source shortest paths for non-negative weights
  - `astar()` - Heuristic-guided pathfinding for faster goal-directed search
  - `bellman_ford()` - Handles negative edge weights, detects negative cycles

- **Graph Traversal**
  - `bfs()` - Breadth-first search with customizable visitor callbacks
  - `dfs()` - Depth-first search with pre/post-order hooks
  - Iterative and recursive implementations available

- **Connectivity Analysis**
  - `connected_components()` - Find connected components in undirected graphs
  - `strongly_connected_components()` - Tarjan's and Kosaraju's algorithms
  - `is_connected()`, `is_bipartite()` - Graph property queries

- **Cycle Detection**
  - Detect cycles in directed and undirected graphs
  - Return the actual cycle path when found
  - Support for self-loops and multi-edges

- **Minimum Spanning Trees**
  - `prim_mst()` - Prim's algorithm for dense graphs
  - `kruskal_mst()` - Kruskal's algorithm with union-find

- **Topological Sorting**
  - `topological_sort_kahn()` - BFS-based, detects cycles
  - `topological_sort_dfs()` - DFS-based implementation
  - `all_topological_sorts()` - Generate all valid orderings

- **Centrality Measures**
  - Degree centrality (in/out/total)
  - Betweenness centrality
  - Closeness centrality
  - PageRank algorithm

- **Graph Transformations**
  - `transpose()` - Reverse all edge directions
  - `subgraph()` - Extract induced subgraph
  - `line_graph()` - Convert edges to vertices
  - Graph views for lazy evaluation

### Factor Graph Optimization

Nonlinear least-squares optimization for robotics and state estimation:

- **SE2 Factors** (2D poses with proper Lie group math)
  - `SE2PriorFactor` - Anchor poses to known values
  - `SE2BetweenFactor` - Relative pose constraints (odometry, loop closures)
  - Uses exp/log maps for manifold-aware error computation

- **Scalar Factors** (1D variables)
  - `PriorFactor` - Simple prior constraints
  - `BetweenFactor` - Relative difference constraints

- **Optimizers**
  - `GaussNewtonOptimizer` - Fast convergence near optimum
  - `LevenbergMarquardtOptimizer` - Robust with adaptive damping
  - `GradientDescentOptimizer` - Simple, supports Adam optimizer

- **Robust Loss Functions**
  - `HuberLoss` - Balanced outlier handling
  - `CauchyLoss` - Aggressive downweighting
  - `TukeyLoss` - Complete outlier rejection
  - `GemanMcClureLoss` - Smooth redescending

- **Linearization**
  - `GaussianFactor` - Linearized factor representation
  - Automatic Jacobian computation via finite differences
  - Efficient sparse matrix assembly

### Modern C++20 Design

- **Header-only** - Just include and use, no linking required
- **Type-safe** - Concepts and constraints prevent misuse at compile time
- **Range-based** - Iterate vertices and edges with range-for loops
- **Zero overhead** - Optional vertex properties add no cost when unused
- **Serialization** - Export to DOT format for visualization

### Performance Characteristics

- **SIMD acceleration** - Math operations use optinum's vectorized implementations
- **Sparse Jacobians** - Only non-zero blocks are computed and stored
- **Adaptive damping** - LM optimizer adjusts lambda based on error improvement
- **Line search** - Gradient descent uses backtracking for step size selection
- **Efficient storage** - Custom `SmallVec` avoids heap allocation for small graphs

## Examples

The `examples/` directory contains comprehensive demonstrations:

### Vertex Graph Examples

| Example | Description |
|---------|-------------|
| `simple_graph.cpp` | Basic vertex graph operations and queries |
| `dijkstra_example.cpp` | Single-source shortest path finding |
| `astar_example.cpp` | A* pathfinding with custom heuristics |
| `bellman_ford_example.cpp` | Shortest paths with negative weights |
| `bfs_example.cpp` | Breadth-first search traversal |
| `cycle_detection_example.cpp` | Finding cycles in graphs |
| `centrality_example.cpp` | Computing graph centrality measures |
| `strongly_connected_components_example.cpp` | SCC decomposition |
| `topological_sort_example.cpp` | Ordering DAG vertices |
| `graph_properties_example.cpp` | Querying graph properties |
| `graph_transformations_example.cpp` | Graph manipulation operations |
| `graph_views_example.cpp` | Lazy graph views and filters |
| `property_map_example.cpp` | External property storage |
| `serialization_example.cpp` | DOT format export |

### Factor Graph Examples

| Example | Description |
|---------|-------------|
| `se2_slam_2d.cpp` | 2D SLAM with SE2 poses and loop closure |
| `robust_slam_example.cpp` | Outlier rejection with robust loss functions |
| `compare_optimizers.cpp` | GN vs LM vs GD performance comparison |

### Building and Running

```bash
# Configure with examples enabled
xmake f --examples=y -y

# Build all
xmake

# Run a specific example
xmake run se2_slam_2d
xmake run dijkstra_example
xmake run robust_slam_example
```

## License

MIT License - see [LICENSE](./LICENSE) for details.

## Acknowledgments

Made possible thanks to [these amazing projects](./ACKNOWLEDGMENTS.md).
