# Factor Graphs

The `graphix::factor` module is the optimization layer.

Most users should start from these public types:

- `Values`
- `PriorFactor`
- `BetweenFactor`
- `SE2PriorFactor`
- `SE2BetweenFactor`
- `PoseGraph2d`
- `GaussNewtonOptimizer`
- `LevenbergMarquardtOptimizer`
- `GradientDescentOptimizer`

For 2D robotics-style workflows, `PoseGraph2d` is the preferred high-level entrypoint.

```rust
use glam::DVec2;
use graphix::{X};
use graphix::factor::{GaussNewtonOptimizer, PoseGraph2d, SE2d};

let mut problem = PoseGraph2d::new();
problem
    .insert_pose(X(0).into(), SE2d::identity())?
    .insert_pose(X(1).into(), SE2d::from_translation_angle(DVec2::new(1.0, 0.0), 0.0))?
    .add_prior(X(0).into(), SE2d::identity(), DVec2::splat(0.01), 0.01)?
    .add_between(
        X(0).into(),
        X(1).into(),
        SE2d::from_translation_angle(DVec2::new(1.0, 0.0), 0.0),
        DVec2::splat(0.05),
        0.03,
    )?;

let result = GaussNewtonOptimizer::new().optimize(problem.graph(), problem.initial_values());
# let _ = result;
# Ok::<(), String>(())
```

If you need direct lower-level control, you can still work with factor graphs and `Values` manually.
