use std::{
    cmp::Reverse,
    collections::{HashMap, HashSet},
};

use geo::{Contains, Coord, Distance, Euclidean, Intersects, Line, MultiPolygon};
use keyed_priority_queue::{Entry, KeyedPriorityQueue};
use ordered_float::OrderedFloat;

use crate::map::ObstacleMap;

#[derive(Debug, Clone)]
pub struct PathPlanner {
    vertices: Vec<Coord>,
    obstacle_critical: MultiPolygon,
    obstacle_with_margin: MultiPolygon,
    start: usize,
    goal: usize,
    open: KeyedPriorityQueue<usize, Reverse<OrderedFloat<f64>>>,
    g_score: HashMap<usize, OrderedFloat<f64>>,
    successors: HashSet<usize>,
    from: HashMap<usize, usize>,
}

const MULTIPLIER: f64 = 10.0;

impl PathPlanner {
    /// Create a new path planner planning task with the given map, start and goal
    pub fn new(map: &ObstacleMap, start: (f64, f64), goal: (f64, f64)) -> Self {
        let mut vertices = map.as_vertices();
        vertices.append(&mut vec![start.into(), goal.into()]);
        let start: usize = vertices.len() - 2;
        let goal = vertices.len() - 1;
        let obstacle_critical = map.as_multipolygon(false);
        let obstacle_with_margin = map.as_multipolygon(true);
        let mut open = KeyedPriorityQueue::<usize, Reverse<OrderedFloat<f64>>>::new();
        open.push(
            start,
            Reverse(OrderedFloat(Euclidean::distance(
                vertices[start],
                vertices[goal],
            ))),
        );
        let from = HashMap::<usize, usize>::new();
        let successors = HashSet::<usize>::from_iter(0..vertices.len());
        let mut g_score = HashMap::<usize, OrderedFloat<f64>>::new();
        g_score.insert(start, OrderedFloat(0.0));
        Self {
            vertices,
            obstacle_critical,
            obstacle_with_margin,
            start,
            goal,
            open,
            g_score,
            successors,
            from,
        }
    }

    /// Part of A* algorithm, calculates the heuristic for the given node.
    /// Often the heuristic is the euclidean distance to the goal.
    fn heuristic(&self, node: usize) -> OrderedFloat<f64> {
        self.distance(node, self.goal)
    }

    /// Calculates the euclidean distance between two vertices
    fn distance(&self, from: usize, to: usize) -> OrderedFloat<f64> {
        OrderedFloat(Euclidean::distance(self.vertices[from], self.vertices[to]))
    }

    /// Part of A* algorithm, reconstructs the path from the start to the goal vertex
    /// based on the predecessors calculated during the search
    fn reconstruct_path(&self, vertex: usize) -> Vec<(f64, f64)> {
        let mut path = vec![vertex];
        let mut current = vertex;
        while let Some(previous) = self.from.get(&current) {
            current = *previous;
            path.push(*previous);
        }
        path.reverse();
        path.into_iter()
            .map(|idx| self.vertices[idx].x_y())
            .collect()
    }

    // Calculates and rates the connections of the vertex
    fn expand_node(&mut self, vertex: usize) {
        let g_vertex = *self
            .g_score
            .get(&vertex)
            .unwrap_or(&OrderedFloat(f64::INFINITY));
        for successor in self.successors.iter().cloned() {
            let connection = Line::new(self.vertices[vertex], self.vertices[successor]);
            // Check if the connection intersects with anything
            let multiplier = if self.obstacle_with_margin.intersects(&connection) {
                // Check if we are at the start or goal
                // We want to always allow connections to the start and goal,
                // but we want to penalize connections that intersect with an obstacles critical area
                // So we don't want to connect the visibility graph through an obstacle except for the start and goal
                if (self.start == vertex && self.goal == successor) // Best effort backup solution, so we allow the direct connection at high cost
                    // Also consider start and goal as inside the obstacle (at high cost if we are inside the critical area, and at low cost if we are inside the margin)
                    || (self.goal == successor && self.obstacle_with_margin.contains(&connection.end_point()))
                    || (self.start == vertex && self.obstacle_with_margin.contains(&connection.start_point())
                ) {
                    // Use high cost for connections that intersect with the critical area of an obstacle
                    if self.obstacle_critical.intersects(&connection) {
                        OrderedFloat(MULTIPLIER)
                    } else {
                        // When we are at the start or goal and the connection is inside the obstacle margin,
                        // but not critical we want to allow this connection without penalty.
                        // This way we avoid situations where the robot gets stuck.
                        // Image a situation where the robot follows the normal path around an obstacle.
                        // Due to the nature of things it's start point is not exactly on the edge.
                        // We therefore are inside the obstacle and get a high cost roughly 50 percent of the time.
                        // This leads to us going back to the vertex we came from
                        // (it has the shortest path though the high cost region) and trying again.
                        // Immediately after departing from the vertex we are inside the obstacle again (~50 percent of the time).
                        // This can lead to the robot getting stuck in a loop.
                        // We therefore add a margin around the obstacle where we don't penalize start and end connections,
                        // while prohibiting normal connections of the visibility graph (to avoid getting too close to the obstacle again).
                        OrderedFloat(1.0)
                    }
                } else {
                    // Ignore connections that intersect with obstacles
                    // (except for the start and goal as they might be inside the obstacle and we do a best effort solution)
                    continue;
                }
            } else {
                // Don't penalize connections that don't intersect with anything
                OrderedFloat(1.0)
            };

            // Part of A* algorithm
            let g_successor = *self
                .g_score
                .get(&successor)
                .unwrap_or(&OrderedFloat(f64::INFINITY));
            let g_tentative = g_vertex + multiplier * self.distance(vertex, successor);
            if g_tentative < g_successor {
                self.from.insert(successor, vertex);
                self.g_score.insert(successor, g_tentative);
                let new_f_score = Reverse(g_tentative + self.heuristic(successor));
                match self.open.entry(successor) {
                    Entry::Occupied(entry) => {
                        entry.set_priority(new_f_score);
                    }
                    Entry::Vacant(entry) => {
                        entry.set_priority(new_f_score);
                    }
                }
            }
        }
    }

    /// Find the shortest path from the start to the goal considering the obstacles
    pub fn shortest_path(mut self) -> Vec<(f64, f64)> {
        while let Some((vertex, _)) = self.open.pop() {
            if vertex == self.goal {
                return self.reconstruct_path(vertex);
            }
            self.successors.remove(&vertex);
            self.expand_node(vertex);
        }
        unreachable!()
    }
}
