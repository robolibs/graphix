# Getting Started

Add the crate to your `Cargo.toml`:

```toml
[dependencies]
graphix = { path = "../graphix_rs" }
glam = "0.30"
```

The library root exports the main namespaces:

```rust
use graphix::{X, L, P};
use graphix::vertex;
use graphix::factor;
```

Useful local commands while working in this repository:

```sh
make run
make test
make bench
```

Canonical examples to inspect first:

- `cargo run --example main`
- `cargo run --example spatial_pathfinding`
- `cargo run --example spatial_pose_graph`
- `cargo run --example robust_slam`
- `cargo run --example compare_optimizers`
