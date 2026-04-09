# API Map

This is the high-level public map of the crate.

## Root

```rust
use graphix::{Id, Key, Symbol, Store, X, L, P};
```

## Vertex

```rust
use graphix::vertex;
use graphix::vertex::Graph;
use graphix::vertex::algorithms;
use graphix::vertex::spatial;
use graphix::vertex::views;
use graphix::vertex::transformations;
use graphix::vertex::property_map;
use graphix::vertex::serialization;
```

## Factor

```rust
use graphix::factor;
use graphix::factor::{
    PoseGraph2d,
    Values,
    SE2d,
    GaussNewtonOptimizer,
    LevenbergMarquardtOptimizer,
    GradientDescentOptimizer,
};
```

## When To Use What

- Use `vertex` when your problem is mostly routing, connectivity, or graph analysis.
- Use `vertex::spatial` when your data starts as 2D points or observations.
- Use `factor` when your problem is an optimization problem over variables and constraints.
- Use `PoseGraph2d` when you are solving 2D pose-graph style problems and want the simplest high-level path.
