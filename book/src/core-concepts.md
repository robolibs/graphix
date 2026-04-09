# Core Concepts

There are two separate graph layers in `graphix`.

## Vertex Graphs

`graphix::vertex::Graph<V, E>` stores:

- vertex properties of type `V`
- edge properties of type `E`
- edge weights as `f64`
- directed or undirected edges

This is the layer you use for:

- traversal
- shortest paths
- graph views and transformations
- spatial graph construction

## Factor Graphs

`graphix::factor` is for optimization problems.

The main public concepts are:

- `Values` for variable assignments
- nonlinear factors such as priors and between-factors
- optimizers such as Gauss-Newton and Levenberg-Marquardt
- `PoseGraph2d` as a higher-level SE2 workflow helper

## Symbols and Keys

The crate root exports helpers such as:

```rust
use graphix::{X, L, P};
```

These are convenient symbolic keys for variables:

- `X(i)` for state/pose-like variables
- `L(i)` for landmark-like variables
- `P(i)` for generic property/parameter-like variables
