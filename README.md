# graphix

`graphix` is a Rust graph library aimed at practical robotics and spatial-graph workflows.

The goal is not one-to-one C++ parity with the upstream `graphix` project. The goal is a useful Rust library with:

- solid vertex-graph algorithms
- spatial graph construction on top of `kiddo`
- factor-graph optimization for realistic 2D pose-graph workflows
- examples and tests that exercise combined graph + factor pipelines

## Scope

`graphix` currently focuses on three areas:

- vertex graphs: traversal, shortest paths, SCC, MST, centrality, graph views and transformations
- spatial graph utilities: nearest-neighbor queries, k-NN/radius graph construction, and correspondence helpers
- factor graphs: scalar and SE2 nonlinear factors, robust losses, and Gauss-Newton / LM / gradient-descent optimizers

Low-level math/container compatibility surface is intentionally de-emphasized. The crate prefers ecosystem primitives where they help:

- `glam` for geometry/math types
- `kiddo` for nearest-neighbor lookup
- `rayon` for explicit parallel graph workloads
- `tokio` for async DOT IO

## Canonical Examples

These are the examples that currently define the intended workflows:

- `cargo run --example main`
  Development entrypoint and broad feature sampler.
- `cargo run --example spatial_pathfinding`
  Build a spatial graph from points and run shortest path.
- `cargo run --example spatial_pose_graph`
  Full spatial graph -> route -> pose-graph optimization pipeline.
- `cargo run --example robust_slam`
  SE2 pose-graph optimization with robust loss functions.
- `cargo run --example compare_optimizers`
  Compare SE2 optimizer behavior on the same problem.

The examples directory is intentionally trimmed to focused workflows rather than alias duplicates.

## Developer Commands

- `make run`
  Runs `cargo run --example main`
- `make test`
  Runs the full test suite
- `make bench`
  Runs the custom quick benchmark target

## Performance

There is a dedicated benchmark target in [benches/core_workloads.rs](./benches/core_workloads.rs).

Run it with:

```sh
make bench
```

or:

```sh
cargo bench --bench core_workloads -- --quick
```

Current quick benchmark numbers from this repository on April 9, 2026:

- spatial k-NN graph build: avg `1.195 ms`
- parallel betweenness centrality: avg `62.219 ms`
- Gauss-Newton pose chain optimization: avg `8.759 ms`

These numbers are only a local reference point. They depend on machine, compiler version, and system load.

One practical detail: `core_workloads` is a custom `harness = false` bench target, so `cargo test --all-targets` also executes it. That is valid, but it makes the all-targets test path slower than `cargo test --tests --examples`.
