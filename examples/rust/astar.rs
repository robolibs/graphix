use graphix::vertex::algorithms::{astar, manhattan_distance};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<(), ()>::new();
    let vertices: Vec<_> = (0..9).map(|_| graph.add_unit_vertex()).collect();

    for row in 0..3 {
        for col in 0..3 {
            let idx = row * 3 + col;
            if col < 2 {
                graph.add_edge(
                    vertices[idx],
                    vertices[idx + 1],
                    1.0,
                    EdgeType::Undirected,
                    (),
                );
            }
            if row < 2 {
                graph.add_edge(
                    vertices[idx],
                    vertices[idx + 3],
                    1.0,
                    EdgeType::Undirected,
                    (),
                );
            }
        }
    }

    let result = astar(&graph, vertices[0], vertices[8], |v, target| {
        manhattan_distance(v.value() as usize, target.value() as usize, 3)
    });
    println!("astar found: {}", result.found);
    println!("astar distance: {}", result.distance);
    println!("astar explored: {}", result.nodes_explored);
}
