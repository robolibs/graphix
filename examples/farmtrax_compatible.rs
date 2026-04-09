use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::{
    Graph, VertexId, add_edge_with_weight, add_vertex_with_property, num_edges, num_vertices,
};

#[derive(Debug, Clone, Copy, PartialEq)]
struct Point2D {
    x: f64,
    y: f64,
}

impl Point2D {
    fn distance_to(self, other: Self) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

struct FieldPathPlanner {
    graph: Graph<Point2D, ()>,
    line_endpoints: Vec<(VertexId<Point2D>, VertexId<Point2D>)>,
}

impl FieldPathPlanner {
    fn new(lines: &[(Point2D, Point2D)]) -> Self {
        let mut graph = Graph::new();
        let mut line_endpoints = Vec::with_capacity(lines.len());

        for (a, b) in lines {
            let va = add_vertex_with_property(*a, &mut graph);
            let vb = add_vertex_with_property(*b, &mut graph);
            add_edge_with_weight(va, vb, a.distance_to(*b), &mut graph);
            line_endpoints.push((va, vb));
        }

        for pair in line_endpoints.windows(2) {
            let (_, prev_end) = pair[0];
            let (next_start, _) = pair[1];
            let from = graph[prev_end];
            let to = graph[next_start];
            add_edge_with_weight(prev_end, next_start, from.distance_to(to), &mut graph);
        }

        Self {
            graph,
            line_endpoints,
        }
    }

    fn shortest_path_between_lines(
        &self,
        start_line: usize,
        goal_line: usize,
    ) -> Option<(f64, Vec<VertexId<Point2D>>)> {
        let start = self.line_endpoints.get(start_line)?.0;
        let goal = self.line_endpoints.get(goal_line)?.1;
        let result = dijkstra(&self.graph, start, goal);
        result.found.then_some((result.distance, result.path))
    }
}

fn main() {
    let lines = vec![
        (Point2D { x: 0.0, y: 0.0 }, Point2D { x: 10.0, y: 0.0 }),
        (Point2D { x: 10.0, y: 0.0 }, Point2D { x: 20.0, y: 0.0 }),
        (Point2D { x: 20.0, y: 0.0 }, Point2D { x: 30.0, y: 5.0 }),
    ];

    let planner = FieldPathPlanner::new(&lines);
    println!(
        "farmtrax-compatible graph: {} vertices, {} edges",
        num_vertices(&planner.graph),
        num_edges(&planner.graph)
    );

    if let Some((distance, path)) = planner.shortest_path_between_lines(0, 2) {
        println!("path distance: {distance:.3}");
        for vertex in path {
            let point = planner.graph[vertex];
            println!("  ({:.1}, {:.1})", point.x, point.y);
        }
    } else {
        println!("no path found");
    }
}
