use std::collections::{HashMap, HashSet};

use geo::{
    BooleanOps, Coord, Distance, Euclidean, Intersects, Line, LineString, MultiPolygon, Polygon
};
use keyed_priority_queue::{Entry, KeyedPriorityQueue};
use ordered_float::OrderedFloat;
use pyo3::prelude::*;

use crate::util::regular_ngon;

/// A trait representing an Obstacle that can be represented as a Polygon
pub trait Obstacle {
    /// Get the Polygon representation of this obstacle
    fn as_polygon(&self, config: &ObstacleMapConfig) -> Polygon;

    /// Get the Polygon representation of this obstacle FOR INTERSECTION
    fn as_polygon_for_intersection(&self, config: &ObstacleMapConfig) -> Polygon;
}

/// A robot obstacle
#[pyclass(str = "Robot(center={center:?}, radius={radius})")]
#[derive(Debug, Copy, Clone)]
pub struct Robot {
    #[pyo3(get, set)]
    center: (f64, f64),
    #[pyo3(get, set)]
    radius: f64,
}

#[pymethods]
impl Robot {
    #[new]
    pub fn new(center: (f64, f64), radius: f64) -> Self {
        Self { center, radius }
    }
}

impl Obstacle for Robot {
    fn as_polygon(&self, config: &ObstacleMapConfig) -> Polygon {
        regular_ngon(self.center.into(), self.radius + config.dilate, 12)
    }

    fn as_polygon_for_intersection(&self, config: &ObstacleMapConfig) -> Polygon {
        regular_ngon(self.center.into(), self.radius + config.dilate - 0.01, 12)
    }
}

#[pyclass(str = "Robot(center={center:?}, radius={radius})")]
#[derive(Debug, Copy, Clone)]
pub struct Ball {
    #[pyo3(get, set)]
    center: (f64, f64),
    #[pyo3(get, set)]
    radius: f64,
}

#[pymethods]
impl Ball {
    #[new]
    pub fn new(center: (f64, f64), radius: f64) -> Self {
        Self { center, radius }
    }
}

impl Obstacle for Ball {
    fn as_polygon(&self, config: &ObstacleMapConfig) -> Polygon {
        regular_ngon(self.center.into(), self.radius + config.dilate, 12)
    }

    fn as_polygon_for_intersection(&self, config: &ObstacleMapConfig) -> Polygon {
        regular_ngon(self.center.into(), self.radius + config.dilate - 0.01, 12)
    }
}
#[pyclass]
#[derive(Debug, Clone, Copy)]
pub struct ObstacleMapConfig {
    #[pyo3(get, set)]
    dilate: f64,
}

#[pymethods]
impl ObstacleMapConfig {
    #[new]
    pub fn new(dilate: f64) -> Self {
        Self { dilate }
    }
}

#[pyclass]
#[derive(Debug, Clone)]
pub struct ObstacleMap {
    config: ObstacleMapConfig,
    #[pyo3(get, set)]
    ball: Option<Ball>,
    #[pyo3(get, set)]
    robots: Vec<Robot>,
}

impl ObstacleMap{
    /// Get the list of polygonal obstacles in the map
    fn polygons(&self) -> Vec<Polygon> {
        let mut obstacles = self
            .robots
            .iter()
            .map(|robot| robot.as_polygon(&self.config))
            .collect::<Vec<Polygon>>();
        if let Some(ball) = self.ball {
            obstacles.push(ball.as_polygon(&self.config));
        };
        obstacles
    }

    /// Get the list of polygonal obstacles in the map for correct intersection
    fn polygons_for_intersection(&self) -> Vec<Polygon> {
        let mut obstacles = self
            .robots
            .iter()
            .map(|robot| robot.as_polygon_for_intersection(&self.config))
            .collect::<Vec<Polygon>>();
        if let Some(ball) = self.ball {
            obstacles.push(ball.as_polygon_for_intersection(&self.config));
        };
        obstacles
    }

    /// Get the visibility graph of this map
    fn vertices_and_obstacles(&self, start: Coord, goal: Coord) -> (Vec<Coord>, MultiPolygon) {
        let polygons = self.polygons();
        let polygons_for_intersection = self.polygons_for_intersection();
        let vertices = polygons
            .iter()
            .map(|polygon| polygon.exterior().coords().cloned())
            .flatten()
            .chain(vec![start, goal].into_iter())
            .collect::<Vec<Coord>>();
        let obstacles = polygons_for_intersection.iter().fold(
            MultiPolygon::new(vec![Polygon::new(LineString::new(vec![]), vec![])]),
            |a, b| a.union(b),
        );
        (vertices, obstacles)
    }

    fn heuristic(vertices: &Vec<Coord>, node: usize) -> OrderedFloat<f64> {
        OrderedFloat(Euclidean::distance(
            vertices[node],
            vertices[vertices.len() - 1],
        ))
    }

    fn distance(vertices: &Vec<Coord>, from: usize, to: usize) -> OrderedFloat<f64> {
        OrderedFloat(Euclidean::distance(vertices[from], vertices[to]))
    }
}

#[pymethods]
impl ObstacleMap {
    /// Create a new ObstacleMap with the given initial ball and robots
    #[new]
    #[pyo3(signature=(config, robots, ball=None))]
    pub fn new(config: ObstacleMapConfig, robots: Vec<Robot>, ball: Option<Ball>) -> Self {
        Self {
            config,
            ball,
            robots,
        }
    }

    /// Get the shortest from start to goal in a given visibilty graph, which defaults to self.visibility_graph
    pub fn shortest_path(&self, start: (f64, f64), goal: (f64, f64)) -> Option<Vec<(f64, f64)>> {
        let (vertices, obstacles) = self.vertices_and_obstacles(start.into(), goal.into());
        let mut successors = HashSet::<usize>::from_iter(0..vertices.len());
        let mut open = KeyedPriorityQueue::<usize, OrderedFloat<f64>>::new();
        open.push(
            vertices.len() - 2,
            -Self::heuristic(&vertices, vertices.len() - 2),
        );
        let mut from = HashMap::<usize, usize>::new();
        let mut g_score = HashMap::<usize, OrderedFloat<f64>>::new();
        g_score.insert(vertices.len() - 2, OrderedFloat(0.0));

        // needed if goal is in obstacle
        let mut closest_vertex = vertices.len() - 2;
        let mut closest_distance = OrderedFloat(std::f64::INFINITY);

        while let Some((vertex, _)) = open.pop() {

            if Self::heuristic(&vertices, vertex) < closest_distance - 0.01
            {
                closest_vertex = vertex;
                closest_distance = Self::heuristic(&vertices, vertex);
            };
            
            if vertex == vertices.len() - 1 {
                let mut path = vec![vertex];
                let mut current = vertex;
                while let Some(previous) = from.get(&current) {
                    current = *previous;
                    path.push(*previous);
                }
                path.reverse();
                return Some(path.into_iter().map(|idx| vertices[idx].x_y()).collect());
            }
            let g_vertex = g_score
                .get(&vertex)
                .unwrap_or(&OrderedFloat(std::f64::INFINITY))
                .clone();
            successors.remove(&vertex);
            //println!("evaluating successors to node {} (coords={:?})", vertex, vertices[vertex]);
            for successor in successors.iter() {
                if obstacles.intersects(&Line::new(vertices[vertex], vertices[*successor])) {
                    //println!("  not considering node {} (coords={:?}) because of intersection", successor, vertices[*successor]);
                    continue;
                }
                //println!("  successor={} coords={:?}", successor, vertices[*successor]);
                let g_successor = g_score
                    .get(successor)
                    .unwrap_or(&OrderedFloat(std::f64::INFINITY));
                let g_tentative = g_vertex + &Self::distance(&vertices, vertex, *successor);
                //println!("  successor={successor}, tentative={g_tentative}, successor={g_successor}");
                if g_tentative < *g_successor {
                    from.insert(*successor, vertex);
                    g_score.insert(*successor, g_tentative);
                    match open.entry(*successor) {
                        Entry::Occupied(entry) => {
                            entry.set_priority(-(g_tentative + Self::heuristic(&vertices, *successor)));
                        },
                        Entry::Vacant(entry) => {
                            entry.set_priority(-(g_tentative + Self::heuristic(&vertices, *successor)));
                        }
                    };
                };
            }
        }
        let mut path = vec![vertices.len() - 1];
                let mut current = closest_vertex;
                path.push(current);
                while let Some(previous) = from.get(&current) {
                    current = *previous;
                    path.push(*previous);
                }
                path.reverse();
                return Some(path.into_iter().map(|idx| vertices[idx].x_y()).collect());
    }
}
