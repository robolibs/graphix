# Changelog

## [0.0.2] - 2025-12-30

### <!-- 0 -->⛰️  Features

- Add OptinumLevenbergMarquardt and OptinumGradientDescent adapters
- Add OptinumGaussNewton to optimizer comparison
- Integrate optinum optimizers with factor graphs
- Add SE2 support to NonlinearFactor::linearize()
- Replace custom matrix and vector types with Datapod
- Add basic type definitions and aliases
- Inline and simplify core graphix components
- Introduce composable graph views for lazy graph adaptations
- Add graph transformation utilities
- Add graph centrality algorithms and examples
- Add graph property algorithms and utilities
- Add A* pathfinding algorithm implementation
- Introduce Boost.Graph-like property maps
- Implement the Bellman-Ford algorithm and its utilities
- Implement cycle detection and SCC algorithms
- Add example for topological sort applications
- Add graph topological sort algorithm
- Add robust loss functions to `gtsam.BetweenFactor` and `gtsam.PriorFactor`
- Add robust loss functions to nonlinear factors
- Implement Levenberg-Marquardt optimizer and compare it
- Implement Gauss-Newton optimizer and SLAM example
- Implement Gaussian and linearized nonlinear factors
- Introduce Matrix class for dense linear algebra
- Implement Adam optimizer and enhance gradient descent
- Feat(SLAM): Add 2D SLAM example and Vec3d support
- Add 3D vector factors and associated error checking
- Add 3D vector class with comprehensive utilities
- Implement and test new gradient descent optimizer
- Introduce nonlinear factor types and associated tests
- Enhance Factor class functionality and test coverage
- Add key iteration and range-based for loop to `Values`
- Generalize `Values` class to support arbitrary types
- Introduce type-erased value storage and container
- Add DOT serialization examples for various graph types
- Implement graph serialization to DOT format
- Implement graph copy and move operations
- Introduce EdgeType for graph edge management
- Implement Prim's algorithm for Minimum Spanning Tree
- Implement graph connected components algorithms
- Add Depth-First Search (DFS) algorithm implementations
- Implement Breadth-First Search (BFS) algorithm and tests
- Add comprehensive graph examples for all features
- Add graph edge query functions
- Add Boost-style free functions for graph operations
- Implement graph clear and element removal methods
- Add Dijkstra's shortest path algorithm for graphs
- Refactor graph iteration to use `EdgeDescriptor` and `all_ids`
- Add graph `neighbors` and `degree` methods
- Implement basic graph edge management
- Introduce templated `Graph` with property management
- Introduce optimized ID and symbol storage
- Introduce core graphix library components
- Init

### <!-- 2 -->🚜 Refactor

- Replace optimizer implementations with thin wrappers
- Migrate from Vec3 to SE2 factors
- Update graph to a templated class and add new features
- Rename `vertice` to `vertex`
- Remove simple example and update environment setup

### <!-- 3 -->📚 Documentation

- Update comment to use SE2BetweenFactor instead of Vec3BetweenFactor
- Update acknowledgments for Boost.Graph and GTSAM
- Add an acknowledgements section for open-source projects
- Docs: Update README with new logo and formatting

### <!-- 6 -->🧪 Testing

- Test graph edge cases and mixed graph functionality
- Test core utility and collection classes

### <!-- 7 -->⚙️ Miscellaneous Tasks

- Add pkg-config dependency support to CMake
- Refactor build system for flexibility

# Changelog
