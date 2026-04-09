# Vertex Graphs

Start with `graphix::vertex::Graph`.

```rust
use graphix::vertex::{EdgeType, Graph};

let mut g = Graph::<&str, ()>::new();
let a = g.add_vertex("a");
let b = g.add_vertex("b");
g.add_edge(a, b, 1.0, EdgeType::Undirected, ());
```

The `vertex` module currently covers:

- BFS and DFS
- Dijkstra and Bellman-Ford
- A*
- connected components
- SCC
- topological sort
- cycle detection
- MST
- graph metrics and centrality
- graph views and transformations
- property maps
- DOT serialization

Common modules:

- `graphix::vertex::algorithms`
- `graphix::vertex::views`
- `graphix::vertex::transformations`
- `graphix::vertex::property_map`
- `graphix::vertex::serialization`

If you want a graph-algorithm-first entrypoint, this is the part of the library to start with.
