use geo::{BooleanOps, Coord, LineString, MultiPolygon, Polygon};
use pyo3::prelude::*;

use crate::{
    obstacle::{Obstacle, RoundObstacle},
    planner::PathPlanner,
};

/// Configuration values for the ObstacleMap, these should be given by ROS parameters
#[pyclass(
    eq,
    str = "ObstacleMapConfig(robot_radius={robot_radius:?}, margin={margin:?}, num_vertices={num_vertices:?})"
)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ObstacleMapConfig {
    /// All obstacles are inflated by this amount, so we can assume the robot itself is a point
    #[pyo3(set, get)]
    pub robot_radius: f64,
    /// Normal plans should not be directly on the edge of an obstacle,
    /// this margin is added if possible
    #[pyo3(set, get)]
    pub margin: f64,
    /// The number of vertices that the polygon representing an obstacle has
    #[pyo3(set, get)]
    pub num_vertices: usize,
}

#[pymethods]
impl ObstacleMapConfig {
    #[new]
    /// Create a new config with the given values
    pub fn new(robot_radius: f64, margin: f64, num_vertices: usize) -> Self {
        Self {
            robot_radius,
            margin,
            num_vertices,
        }
    }
}

#[pyclass(eq, str = "ObstacleMap(obstacles={obstacles:?})")]
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleMap {
    #[pyo3(get, set)]
    config: ObstacleMapConfig,
    #[pyo3(get, set)]
    obstacles: Vec<RoundObstacle>,
}

#[pymethods]
impl ObstacleMap {
    #[new]
    pub fn new(config: ObstacleMapConfig, obstacles: Vec<RoundObstacle>) -> Self {
        Self { config, obstacles }
    }

    pub fn shortest_path(&self, start: (f64, f64), goal: (f64, f64)) -> Vec<(f64, f64)> {
        PathPlanner::new(self, start, goal).shortest_path()
    }
}

impl ObstacleMap {
    pub fn as_vertices(&self) -> Vec<Coord> {
        self.obstacles
            .iter()
            .flat_map(|obstacle| obstacle.as_vertices(&self.config))
            .collect()
    }

    fn as_polygons(&self, margin: bool) -> Vec<Polygon> {
        self.obstacles
            .iter()
            .map(|obstacle| obstacle.as_polygon(&self.config, margin))
            .collect()
    }

    pub fn as_multipolygon(&self, margin: bool) -> MultiPolygon {
        let polygons = self.as_polygons(margin);
        polygons.iter().fold(
            MultiPolygon::new(vec![Polygon::new(LineString::new(vec![]), vec![])]),
            |a, b| a.union(b),
        )
    }
}
