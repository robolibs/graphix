use std::rc::Rc;

use graphix::X;
use graphix::factor::{
    BetweenFactor, FactorGraphAdapter, GaussNewtonOptimizer, GradientDescentOptimizer, Graph,
    LevenbergMarquardtOptimizer, NonlinearFactor, PriorFactor, Values,
};

fn main() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 0.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    graph.add(
        Rc::new(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );
    graph.add(
        Rc::new(BetweenFactor::new(X(1).into(), X(2).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );

    let mut initial = Values::new();
    initial.insert(X(0).into(), 5.0).unwrap();
    initial.insert(X(1).into(), 5.0).unwrap();
    initial.insert(X(2).into(), 5.0).unwrap();

    let gd = GradientDescentOptimizer::new().optimize(&graph, &initial);
    let gn = GaussNewtonOptimizer::new().optimize(&graph, &initial);
    let lm = LevenbergMarquardtOptimizer::new().optimize(&graph, &initial);
    let adapter = FactorGraphAdapter::new(&graph, &initial).unwrap();

    println!("gd final error: {:.6}", gd.final_error);
    println!("gn final error: {:.6}", gn.final_error);
    println!("lm final error: {:.6}", lm.final_error);
    println!(
        "adapter dims: {} params, {} residuals",
        adapter.param_dim(),
        adapter.residual_dim()
    );
}
