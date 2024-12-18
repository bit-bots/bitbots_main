use geo::{BooleanOps, Coord, LineString, MultiPolygon, Polygon};
use pyo3::prelude::*;

use crate::{obstacle::{Obstacle, RoundObstacle}, planner::PathPlanner};

/// Configuration values for the ObstacleMap, these should be given by ROS parameters
#[pyclass(
    eq,
    str = "ObstacleMapConfig(dilate={dilate:?} num_vertices={num_vertices:?})"
)]
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct ObstacleMapConfig {
    /// A distance by which the obstacles should be dilated
    #[pyo3(set, get)]
    pub dilate: f64,
    /// The number of vertices that the polygon representing an obstacle has
    #[pyo3(set, get)]
    pub num_vertices: usize,
}

#[pymethods]
impl ObstacleMapConfig {
    #[new]
    /// Create a new config with the given values
    pub fn new(dilate: f64, num_vertices: usize) -> Self {
        Self {
            dilate,
            num_vertices,
        }
    }
}

#[pyclass(eq, str = "ObstacleMap(obstacles={obstacles:?})")]
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleMap {
    config: ObstacleMapConfig,
    obstacles: Vec<RoundObstacle>,
}

#[pymethods]
impl ObstacleMap {
    #[new]
    pub fn new(config: ObstacleMapConfig, obstacles: Vec<RoundObstacle>) -> Self {
        Self { config, obstacles }
    }

    pub fn shortest_path(&self, start: (f64, f64), goal: (f64, f64)) -> Vec<(f64, f64)> {
        PathPlanner::new(&self, start, goal).shortest_path()
    }
}

impl ObstacleMap {
    pub fn as_vertices(&self) -> Vec<Coord> {
        self.obstacles
            .iter()
            .map(|obstacle| obstacle.as_vertices(&self.config))
            .flatten()
            .collect()
    }

    fn as_polygons(&self) -> Vec<Polygon> {
        self.obstacles
            .iter()
            .map(|obstacle| obstacle.as_polygon(&self.config))
            .collect()
    }

    pub fn as_multipolygon(&self) -> MultiPolygon {
        let polygons = self.as_polygons();
        polygons.iter().fold(
            MultiPolygon::new(vec![Polygon::new(LineString::new(vec![]), vec![])]),
            |a, b| a.union(b),
        )
    }
}
