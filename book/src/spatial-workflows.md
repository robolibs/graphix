# Spatial Workflows

`graphix::vertex::spatial` provides the 2D spatial layer on top of `kiddo`.

Typical entrypoints:

- `nearest_vertex_2d`
- `k_nearest_vertices_2d`
- `vertices_within_radius_2d`
- `knn_graph_2d`
- `radius_graph_2d`
- `connect_k_nearest_neighbors_2d`
- `connect_vertices_within_radius_2d`
- `nearest_neighbor_correspondences_2d`
- `radius_limited_correspondences_2d`
- `mutual_nearest_correspondences_2d`

Example shape:

```rust
use glam::DVec2;
use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::spatial::knn_graph_2d;

let points = vec![
    DVec2::new(0.0, 0.0),
    DVec2::new(1.0, 0.1),
    DVec2::new(2.0, 0.0),
];

let graph = knn_graph_2d(points, 2, |p| *p)?;
let start = graphix::vertex::VertexId::new(0);
let goal = graphix::vertex::VertexId::new(2);
let path = dijkstra(&graph, start, goal);
# Ok::<(), String>(())
```

This is the layer to use when your source data starts as point sets or observed 2D positions.
