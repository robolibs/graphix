# Integrated Pipelines

The most important design point in `graphix` is that vertex graphs and factor graphs can be used together.

The intended workflow looks like this:

1. Build or query a spatial graph from points
2. Extract a route or a set of correspondences
3. Convert that result into factor-graph variables and constraints
4. Optimize the resulting SE2 problem

The main reference example is:

```sh
cargo run --example spatial_pose_graph
```

That example does all of the following in one flow:

- builds a k-NN graph
- snaps start/goal queries
- finds a route with Dijkstra
- creates a `PoseGraph2d`
- seeds noisy poses
- adds SE2 prior and between constraints
- optimizes the route with Gauss-Newton

If your application mixes geometry, routing, and optimization, start from that example.
