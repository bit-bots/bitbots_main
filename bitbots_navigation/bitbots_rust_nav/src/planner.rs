use std::{cmp::Reverse, collections::{HashMap, HashSet}};

use geo::{Coord, Distance, Euclidean, Intersects, Line, MultiPolygon};
use keyed_priority_queue::{Entry, KeyedPriorityQueue};
use ordered_float::OrderedFloat;

use crate::map::ObstacleMap;

#[derive(Debug, Clone)]
pub struct PathPlanner {
    vertices: Vec<Coord>,
    obstacle: MultiPolygon,
    start: usize,
    goal: usize,
    open: KeyedPriorityQueue<usize, Reverse<OrderedFloat<f64>>>,
    g_score: HashMap<usize, OrderedFloat<f64>>,
    successors: HashSet<usize>,
    from: HashMap<usize, usize>,
}

const MULTIPLIER: f64 = 10.0;


impl PathPlanner {
    pub fn new(map: &ObstacleMap, start: (f64, f64), goal: (f64, f64)) -> Self {
        let mut vertices = map.as_vertices();
        vertices.append(&mut vec![start.into(), goal.into()]);
        let obstacle = map.as_multipolygon();
        let start = vertices.len() - 2;
        let goal = vertices.len() - 1;
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
            obstacle,
            start,
            goal,
            open,
            g_score,
            successors,
            from,
        }
    }

    fn heuristic(&self, node: usize) -> OrderedFloat<f64> {
        self.distance(node, self.goal)
    }

    fn distance(&self, from: usize, to: usize) -> OrderedFloat<f64> {
        OrderedFloat(Euclidean::distance(self.vertices[from], self.vertices[to]))
    }

    fn reconstruct_path(&self, vertex: usize) -> Vec<(f64, f64)> {
        let mut path = vec![vertex];
        let mut current = vertex;
        while let Some(previous) = self.from.get(&current) {
            current = *previous;
            path.push(*previous);
        }
        path.reverse();
        return path
            .into_iter()
            .map(|idx| self.vertices[idx].x_y())
            .collect();
    }

    fn expand_node(&mut self, vertex: usize) {
        let g_vertex = self
                .g_score
                .get(&vertex)
                .unwrap_or(&OrderedFloat(std::f64::INFINITY))
                .clone();
        for successor in self.successors.iter().cloned() {
            let multiplier = if self.obstacle.intersects(&Line::new(self.vertices[vertex], self.vertices[successor])) {
                if self.goal == successor || self.start == vertex {
                    OrderedFloat(MULTIPLIER)
                } else {
                    continue;
                }
            } else {
                OrderedFloat(1.0)
            };
            let g_successor = self.g_score.get(&successor).unwrap_or(&OrderedFloat(std::f64::INFINITY)).clone();
            let g_tentative = g_vertex + multiplier * self.distance(vertex, successor);
            if g_tentative < g_successor {
                self.from.insert(successor, vertex);
                self.g_score.insert(successor, g_tentative);
                let new_f_score = Reverse(g_tentative + self.heuristic(successor));
                match self.open.entry(successor) {
                    Entry::Occupied(entry) => {
                        entry.set_priority(new_f_score);
                    },
                    Entry::Vacant(entry) => {
                        entry.set_priority(new_f_score);
                    },
                }
            }
        }
    }

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
